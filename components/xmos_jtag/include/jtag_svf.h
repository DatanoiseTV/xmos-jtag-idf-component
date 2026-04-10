/*
 * SVF (Serial Vector Format) file player.
 *
 * Parses and executes SVF files through the generic JTAG transport.
 * Supports: SIR, SDR, RUNTEST, STATE, FREQUENCY, ENDDR, ENDIR,
 *           HDR, HIR, TDR, TIR, TRST.
 *
 * Large SVF files are streamed from a PSRAM-backed buffer --
 * no need to fit the entire parsed representation in RAM.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Progress callback -- called periodically during SVF playback. */
typedef void (*svf_progress_cb_t)(size_t bytes_processed, size_t total_bytes,
                                  size_t commands_executed, void *user_ctx);

/** SVF playback options. */
typedef struct {
    svf_progress_cb_t progress_cb;   /**< Progress callback, or NULL */
    void             *user_ctx;      /**< Passed to callback */
    bool              stop_on_mismatch; /**< Abort if TDO doesn't match expected */
} svf_config_t;

/** SVF playback result. */
typedef struct {
    size_t commands_executed;
    size_t tdo_mismatches;
    size_t bytes_processed;
    esp_err_t error;               /**< ESP_OK if completed successfully */
} svf_result_t;

/**
 * Play an SVF file through a JTAG handle.
 *
 * @param jtag_handle  Opaque JTAG handle (xmos_jtag_handle_t or similar)
 * @param svf_data     SVF file contents in memory (ASCII text)
 * @param svf_len      Length in bytes
 * @param config       Playback options (NULL for defaults)
 * @param result       Output result (NULL to ignore)
 */
esp_err_t svf_play(void *jtag_handle,
                   const char *svf_data, size_t svf_len,
                   const svf_config_t *config,
                   svf_result_t *result);

#ifdef __cplusplus
}
#endif
