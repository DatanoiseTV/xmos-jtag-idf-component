/*
 * Lattice iCE40 FPGA programmer.
 *
 * Programs iCE40UP5K (and other iCE40 variants) via SPI slave interface.
 * iCE40 does NOT use JTAG for bitstream loading -- it uses SPI.
 *
 * Supports:
 *   - CRAM loading (volatile, bitstream runs from SRAM)
 *   - SPI flash programming (non-volatile, boots on power-up)
 *
 * Reference: Lattice TN1248 "iCE40 Programming and Configuration"
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** iCE40 SPI + control pin configuration. */
typedef struct {
    gpio_num_t spi_cs;      /**< SPI chip select (directly to iCE40 SPI_SS) */
    gpio_num_t spi_clk;     /**< SPI clock */
    gpio_num_t spi_mosi;    /**< SPI MOSI (to iCE40 SPI_SI) */
    gpio_num_t spi_miso;    /**< SPI MISO (from iCE40 SPI_SO), or GPIO_NUM_NC */
    gpio_num_t creset;      /**< CRESET_B pin (active low reset) */
    gpio_num_t cdone;       /**< CDONE pin (high when configured), or GPIO_NUM_NC */
} ice40_pins_t;

/**
 * Program iCE40 CRAM (volatile).
 *
 * Loads a bitstream into the iCE40's configuration SRAM via SPI slave mode.
 * The bitstream is lost on power cycle. This is the fastest way to load
 * a design during development.
 *
 * Sequence (TN1248 Appendix A):
 *   1. Assert CRESET_B low, assert SPI_SS low
 *   2. Release CRESET_B, wait 1.2 ms for CRAM clear
 *   3. Send bitstream bytes (MSB first per byte)
 *   4. Send 100 dummy clocks
 *   5. Release SPI_SS
 *   6. Wait for CDONE to go high
 *
 * @param pins        Pin configuration
 * @param bitstream   Raw bitstream data (.bin file from icepack)
 * @param length      Length in bytes
 * @param timeout_ms  Timeout waiting for CDONE (0 = skip check)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if CDONE didn't go high
 */
esp_err_t ice40_program_cram(const ice40_pins_t *pins,
                             const uint8_t *bitstream, size_t length,
                             uint32_t timeout_ms);

/**
 * Program iCE40 SPI flash (non-volatile).
 *
 * Holds the iCE40 in reset, writes the bitstream to the external SPI flash
 * chip, then releases reset so the FPGA boots from flash.
 *
 * @param pins        Pin configuration (same SPI bus as the flash chip)
 * @param bitstream   Raw bitstream data
 * @param length      Length in bytes
 * @param offset      Flash offset (usually 0)
 */
esp_err_t ice40_program_flash(const ice40_pins_t *pins,
                              const uint8_t *bitstream, size_t length,
                              uint32_t offset);

/**
 * Reset the iCE40 (toggle CRESET_B).
 */
esp_err_t ice40_reset(const ice40_pins_t *pins, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
