/*
 * SVF (Serial Vector Format) file player.
 *
 * Reference: ASSET InterTech "Serial Vector Format Specification"
 * Also informed by openFPGALoader svf_jtag.cpp (Apache-2.0).
 *
 * The player parses SVF line-by-line from a memory buffer (typically PSRAM)
 * and executes each command through the JTAG transport layer.
 */

#include "jtag_svf.h"
#include "jtag_transport.h"
#include "xmos_jtag.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static const char *TAG = "svf";

/* Access the transport from the opaque handle */
struct xmos_jtag_ctx;
extern struct jtag_transport *xmos_jtag_get_transport(void *handle);

/* -------------------------------------------------------------------------
 * Internal types
 * ---------------------------------------------------------------------- */

/* Remembered XYR data (SIR/SDR/HIR/HDR/TIR/TDR) */
typedef struct {
    uint32_t len;
    uint8_t *tdi;
    uint8_t *tdo;
    uint8_t *mask;
    size_t   alloc;   /* allocated byte size */
} svf_xyr_t;

typedef struct {
    jtag_transport_t *transport;
    /* SVF state */
    int enddr;   /* 0=IDLE, 1=DRPAUSE */
    int endir;   /* 0=IDLE, 1=IRPAUSE */
    /* Remembered data */
    svf_xyr_t hdr, hir, tdr, tir, sdr, sir;
    /* Stats */
    size_t commands;
    size_t mismatches;
    /* Config */
    bool stop_on_mismatch;
} svf_ctx_t;

/* -------------------------------------------------------------------------
 * Hex parsing
 * ---------------------------------------------------------------------- */

static void xyr_free(svf_xyr_t *x)
{
    free(x->tdi); free(x->tdo); free(x->mask);
    memset(x, 0, sizeof(*x));
}

static esp_err_t xyr_resize(svf_xyr_t *x, uint32_t bits)
{
    size_t bytes = (bits + 7) / 8;
    if (bytes > x->alloc) {
        free(x->tdi); free(x->tdo); free(x->mask);
        x->tdi = calloc(1, bytes);
        x->tdo = calloc(1, bytes);
        x->mask = calloc(1, bytes);
        if (!x->tdi || !x->tdo || !x->mask) return ESP_ERR_NO_MEM;
        x->alloc = bytes;
    }
    if (bits != x->len) {
        memset(x->tdi, 0, bytes);
        memset(x->tdo, 0, bytes);
        memset(x->mask, 0xFF, bytes);  /* default mask = all care */
    }
    x->len = bits;
    return ESP_OK;
}

/* Parse hex string like "DEADBEEF" into byte array (LSB at index 0).
 * SVF hex is MSB-first in the string. */
static void parse_hex_to_bytes(const char *hex, size_t hex_len,
                               uint8_t *out, size_t out_bytes)
{
    memset(out, 0, out_bytes);
    /* Process from the end of the hex string */
    int byte_idx = 0, nibble = 0;
    for (int i = (int)hex_len - 1; i >= 0 && byte_idx < (int)out_bytes; i--) {
        char c = hex[i];
        uint8_t v;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else continue;  /* skip whitespace */

        if (nibble == 0) {
            out[byte_idx] = v;
            nibble = 1;
        } else {
            out[byte_idx] |= (v << 4);
            nibble = 0;
            byte_idx++;
        }
    }
}

/* -------------------------------------------------------------------------
 * Token parser -- extract next whitespace-delimited token
 * ---------------------------------------------------------------------- */
static const char *skip_ws(const char *p, const char *end)
{
    while (p < end && isspace((unsigned char)*p)) p++;
    return p;
}

static const char *next_token(const char *p, const char *end,
                              const char **tok_start, size_t *tok_len)
{
    p = skip_ws(p, end);
    *tok_start = p;
    while (p < end && !isspace((unsigned char)*p) && *p != ';' && *p != '(') p++;
    *tok_len = p - *tok_start;
    return p;
}

/* Extract hex data between parentheses: (DEADBEEF) */
static const char *parse_paren_hex(const char *p, const char *end,
                                   uint8_t *out, size_t out_bytes)
{
    p = skip_ws(p, end);
    if (p < end && *p == '(') p++;

    /* Collect all hex chars until ')' */
    const char *hex_start = p;
    while (p < end && *p != ')') p++;
    size_t hex_len = p - hex_start;
    if (p < end && *p == ')') p++;

    parse_hex_to_bytes(hex_start, hex_len, out, out_bytes);
    return p;
}

/* -------------------------------------------------------------------------
 * SVF command handlers
 * ---------------------------------------------------------------------- */

static esp_err_t handle_xyr(svf_ctx_t *ctx, const char *p, const char *end,
                            svf_xyr_t *xyr, int shift_type)
{
    /* shift_type: 0=SIR, 1=SDR, -1=header/trailer (don't shift) */
    const char *tok; size_t tlen;
    p = next_token(p, end, &tok, &tlen);  /* length field */
    uint32_t len = (uint32_t)strtoul(tok, NULL, 10);

    esp_err_t err = xyr_resize(xyr, len);
    if (err != ESP_OK) return err;
    if (len == 0) return ESP_OK;

    size_t bytes = (len + 7) / 8;

    /* Parse optional TDI, TDO, MASK, SMASK */
    while (p < end) {
        p = next_token(p, end, &tok, &tlen);
        if (tlen == 0) break;

        if (tlen == 3 && strncasecmp(tok, "TDI", 3) == 0) {
            p = parse_paren_hex(p, end, xyr->tdi, bytes);
        } else if (tlen == 3 && strncasecmp(tok, "TDO", 3) == 0) {
            p = parse_paren_hex(p, end, xyr->tdo, bytes);
        } else if (tlen == 4 && strncasecmp(tok, "MASK", 4) == 0) {
            p = parse_paren_hex(p, end, xyr->mask, bytes);
        } else if (tlen == 5 && strncasecmp(tok, "SMASK", 5) == 0) {
            /* SMASK: mask for TDI -- we apply it but don't store separately */
            uint8_t *smask = calloc(1, bytes);
            if (smask) {
                p = parse_paren_hex(p, end, smask, bytes);
                for (size_t i = 0; i < bytes; i++)
                    xyr->tdi[i] &= smask[i];
                free(smask);
            }
        }
    }

    /* Execute the shift if this is SIR or SDR */
    if (shift_type >= 0 && len > 0) {
        uint8_t *tdo_buf = calloc(1, bytes);
        if (!tdo_buf) return ESP_ERR_NO_MEM;

        if (shift_type == 0) {
            err = ctx->transport->shift_ir(ctx->transport,
                                           xyr->tdi, tdo_buf, len);
        } else {
            err = ctx->transport->shift_dr(ctx->transport,
                                           xyr->tdi, tdo_buf, len);
        }

        if (err != ESP_OK) {
            free(tdo_buf);
            return err;
        }

        /* Check TDO against expected value with mask */
        bool has_tdo_check = false;
        for (size_t i = 0; i < bytes; i++) {
            if (xyr->tdo[i] != 0) { has_tdo_check = true; break; }
        }

        if (has_tdo_check) {
            for (size_t i = 0; i < bytes; i++) {
                if ((tdo_buf[i] ^ xyr->tdo[i]) & xyr->mask[i]) {
                    ctx->mismatches++;
                    ESP_LOGD(TAG, "TDO mismatch at byte %zu: got 0x%02x, expected 0x%02x (mask 0x%02x)",
                             i, tdo_buf[i], xyr->tdo[i], xyr->mask[i]);
                    if (ctx->stop_on_mismatch) {
                        free(tdo_buf);
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                    break;
                }
            }
        }
        free(tdo_buf);
    }

    return ESP_OK;
}

static esp_err_t handle_runtest(svf_ctx_t *ctx, const char *p, const char *end)
{
    const char *tok; size_t tlen;
    unsigned int clocks = 0;
    double seconds = 0;

    while (p < end) {
        p = next_token(p, end, &tok, &tlen);
        if (tlen == 0) break;

        /* Check for state name (IDLE, DRPAUSE, etc.) -- skip */
        if (isalpha((unsigned char)tok[0]) && tlen > 2) {
            continue;
        }

        /* Number followed by TCK or SEC */
        if (isdigit((unsigned char)tok[0]) || tok[0] == '.') {
            double val = strtod(tok, NULL);
            const char *unit_tok; size_t unit_len;
            p = next_token(p, end, &unit_tok, &unit_len);
            if (unit_len >= 3 && strncasecmp(unit_tok, "TCK", 3) == 0) {
                clocks = (unsigned int)val;
            } else if (unit_len >= 3 && strncasecmp(unit_tok, "SEC", 3) == 0) {
                seconds = val;
            }
        }
    }

    if (clocks > 0) {
        ctx->transport->idle(ctx->transport, clocks);
    }
    if (seconds > 0) {
        esp_rom_delay_us((uint32_t)(seconds * 1e6));
    }

    return ESP_OK;
}

static esp_err_t handle_state(svf_ctx_t *ctx, const char *p, const char *end)
{
    const char *tok; size_t tlen;
    p = next_token(p, end, &tok, &tlen);

    /* We only handle RESET and IDLE transitions */
    if (tlen >= 5 && strncasecmp(tok, "RESET", 5) == 0) {
        return ctx->transport->reset(ctx->transport);
    }
    /* IDLE: just clock once with TMS=0 (we're likely already in RTI) */
    if (tlen >= 4 && strncasecmp(tok, "IDLE", 4) == 0) {
        return ctx->transport->idle(ctx->transport, 1);
    }
    /* Other states: we'd need full TAP state tracking. For now, ignore. */
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Main SVF parser loop
 * ---------------------------------------------------------------------- */

/* We need access to the transport from the opaque handle.
 * Since the handle is xmos_jtag_ctx, and transport is the first useful field,
 * we define a minimal accessor. */
typedef struct {
    jtag_transport_t *transport;
} jtag_handle_peek_t;

esp_err_t svf_play(void *jtag_handle,
                   const char *svf_data, size_t svf_len,
                   const svf_config_t *config,
                   svf_result_t *result)
{
    if (!jtag_handle || !svf_data || svf_len == 0)
        return ESP_ERR_INVALID_ARG;

    jtag_handle_peek_t *peek = (jtag_handle_peek_t *)jtag_handle;

    svf_ctx_t ctx = {
        .transport = peek->transport,
        .enddr = 0,
        .endir = 0,
        .stop_on_mismatch = config ? config->stop_on_mismatch : false,
    };

    const char *p = svf_data;
    const char *end = svf_data + svf_len;

    /* Line buffer for multi-line statements (SVF statements end with ;) */
    char *stmt = malloc(8192);
    if (!stmt) return ESP_ERR_NO_MEM;
    size_t stmt_len = 0;
    esp_err_t err = ESP_OK;

    /* Reset TAP before playing */
    ctx.transport->reset(ctx.transport);

    while (p < end && err == ESP_OK) {
        /* Skip to next non-whitespace */
        while (p < end && isspace((unsigned char)*p)) p++;
        if (p >= end) break;

        /* Skip comments (! or //) */
        if (*p == '!' || (p + 1 < end && p[0] == '/' && p[1] == '/')) {
            while (p < end && *p != '\n') p++;
            continue;
        }

        /* Read until semicolon (statement terminator) */
        while (p < end && *p != ';') {
            if (*p == '!' || (p + 1 < end && p[0] == '/' && p[1] == '/')) {
                /* Skip inline comment */
                while (p < end && *p != '\n') p++;
                continue;
            }
            if (stmt_len < 8191) {
                stmt[stmt_len++] = (char)toupper((unsigned char)*p);
            }
            p++;
        }
        if (p < end && *p == ';') p++;  /* consume semicolon */

        if (stmt_len == 0) continue;
        stmt[stmt_len] = '\0';

        /* Parse the first token (command) */
        const char *s = stmt;
        const char *s_end = stmt + stmt_len;
        const char *cmd; size_t cmd_len;
        s = next_token(s, s_end, &cmd, &cmd_len);

        if (cmd_len == 3 && memcmp(cmd, "SIR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.sir, 0);
        } else if (cmd_len == 3 && memcmp(cmd, "SDR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.sdr, 1);
        } else if (cmd_len == 3 && memcmp(cmd, "HIR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.hir, -1);
        } else if (cmd_len == 3 && memcmp(cmd, "HDR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.hdr, -1);
        } else if (cmd_len == 3 && memcmp(cmd, "TIR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.tir, -1);
        } else if (cmd_len == 3 && memcmp(cmd, "TDR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.tdr, -1);
        } else if (cmd_len == 7 && memcmp(cmd, "RUNTEST", 7) == 0) {
            err = handle_runtest(&ctx, s, s_end);
        } else if (cmd_len == 5 && memcmp(cmd, "STATE", 5) == 0) {
            err = handle_state(&ctx, s, s_end);
        } else if (cmd_len == 5 && memcmp(cmd, "ENDDR", 5) == 0) {
            s = next_token(s, s_end, &cmd, &cmd_len);
            ctx.enddr = (cmd_len >= 4 && memcmp(cmd, "DRPA", 4) == 0) ? 1 : 0;
        } else if (cmd_len == 5 && memcmp(cmd, "ENDIR", 5) == 0) {
            s = next_token(s, s_end, &cmd, &cmd_len);
            ctx.endir = (cmd_len >= 4 && memcmp(cmd, "IRPA", 4) == 0) ? 1 : 0;
        } else if (cmd_len == 9 && memcmp(cmd, "FREQUENCY", 9) == 0) {
            /* FREQUENCY <hz> HZ; -- informational, we can't change TCK dynamically */
            ESP_LOGD(TAG, "SVF FREQUENCY: %s", s);
        } else if (cmd_len == 4 && memcmp(cmd, "TRST", 4) == 0) {
            s = next_token(s, s_end, &cmd, &cmd_len);
            if (cmd_len >= 2 && memcmp(cmd, "ON", 2) == 0) {
                ctx.transport->reset(ctx.transport);
            }
        } else {
            ESP_LOGD(TAG, "SVF: ignoring '%.*s'", (int)cmd_len, cmd);
        }

        ctx.commands++;
        stmt_len = 0;

        /* Progress callback */
        if (config && config->progress_cb && (ctx.commands % 100) == 0) {
            size_t processed = (size_t)(p - svf_data);
            config->progress_cb(processed, svf_len, ctx.commands, config->user_ctx);
        }
    }

    /* Final progress */
    if (config && config->progress_cb) {
        config->progress_cb(svf_len, svf_len, ctx.commands, config->user_ctx);
    }

    /* Cleanup */
    xyr_free(&ctx.hdr); xyr_free(&ctx.hir);
    xyr_free(&ctx.tdr); xyr_free(&ctx.tir);
    xyr_free(&ctx.sdr); xyr_free(&ctx.sir);
    free(stmt);

    if (result) {
        result->commands_executed = ctx.commands;
        result->tdo_mismatches = ctx.mismatches;
        result->bytes_processed = svf_len;
        result->error = err;
    }

    ESP_LOGI(TAG, "SVF done: %zu commands, %zu mismatches, %s",
             ctx.commands, ctx.mismatches,
             err == ESP_OK ? "OK" : esp_err_to_name(err));

    return err;
}
