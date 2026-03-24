/**
 * @file lis3dhtr.h
 * @brief Minimal LIS3DHTR accelerometer driver for ESP32-S3 (I2C)
 *
 * Only the registers and functions needed for the TCU V2 board test are
 * exposed here.  Full register map can be extended as needed.
 *
 * Hardware connections (TCU V2):
 *   SCL  → IO47  (4.7 kΩ pull-up to 3.3 V)
 *   SDA  → IO48  (4.7 kΩ pull-up to 3.3 V)
 *   SA0  → GND   → I2C address = 0x18
 *   INT1 → IO7   (not used in this test — wired for future use)
 *   INT2 → NC
 *
 * I2C address:
 *   SA0 = 0  →  0x18
 *   SA0 = 1  →  0x19
 */

#ifndef LIS3DHTR_H
#define LIS3DHTR_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// I2C BUS CONFIGURATION
// ============================================================================

#define LIS3DHTR_I2C_PORT       I2C_NUM_0
#define LIS3DHTR_SCL_PIN        GPIO_NUM_47
#define LIS3DHTR_SDA_PIN        GPIO_NUM_48
#define LIS3DHTR_I2C_FREQ_HZ    100000          // 100 kHz standard mode

/** I2C address with SA0 tied to GND */
#define LIS3DHTR_I2C_ADDR       0x18

// ============================================================================
// REGISTER MAP (subset)
// ============================================================================

#define LIS3DHTR_REG_WHO_AM_I   0x0F    /**< Device ID register             */
#define LIS3DHTR_REG_CTRL_REG1  0x20    /**< ODR, power mode, axis enable   */
#define LIS3DHTR_REG_CTRL_REG4  0x23    /**< Full-scale, high-res mode      */
#define LIS3DHTR_REG_STATUS_REG 0x27    /**< Data-ready flags               */
#define LIS3DHTR_REG_OUT_X_L    0x28    /**< X-axis low byte                */
#define LIS3DHTR_REG_OUT_X_H    0x29
#define LIS3DHTR_REG_OUT_Y_L    0x2A
#define LIS3DHTR_REG_OUT_Y_H    0x2B
#define LIS3DHTR_REG_OUT_Z_L    0x2C
#define LIS3DHTR_REG_OUT_Z_H    0x2D

/** Expected WHO_AM_I value — confirms correct device on bus */
#define LIS3DHTR_WHO_AM_I_VAL   0x33

// ============================================================================
// CTRL_REG1 helper values
// ============================================================================

/**
 * CTRL_REG1 bit layout:
 *   [7:4] ODR3:ODR0  — Output data rate
 *   [3]   LPen       — Low-power enable
 *   [2]   Zen        — Z-axis enable
 *   [1]   Yen        — Y-axis enable
 *   [0]   Xen        — X-axis enable
 *
 * 0x57 = ODR=100 Hz, normal mode, X/Y/Z enabled
 */
#define LIS3DHTR_CTRL_REG1_100HZ_NORMAL  0x57

// ============================================================================
// DATA STRUCTURE
// ============================================================================

typedef struct {
    int16_t x;  /**< Raw X acceleration (16-bit, 2's complement) */
    int16_t y;  /**< Raw Y acceleration */
    int16_t z;  /**< Raw Z acceleration */
} lis3dhtr_accel_t;

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialise the I2C bus and configure the LIS3DHTR.
 *
 * Installs the I2C driver, verifies WHO_AM_I, then writes CTRL_REG1 to
 * start continuous measurement at 100 Hz with all axes enabled.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if WHO_AM_I mismatch,
 *         or other esp_err_t on I2C failure.
 */
esp_err_t lis3dhtr_init(void);

/**
 * @brief Run an I2C bus scan and log all responding addresses.
 *
 * Probes every 7-bit address from 0x01 to 0x7E and logs any that ACK.
 * Useful for confirming the LIS3DHTR is visible on the bus before
 * attempting register-level communication.
 */
void lis3dhtr_i2c_scan(void);

/**
 * @brief Read the WHO_AM_I register and verify it equals 0x33.
 *
 * @param[out] value  Byte read from the register (0x33 if device is present)
 * @return ESP_OK if read succeeded and value == 0x33, ESP_ERR_INVALID_RESPONSE
 *         if value is wrong, or esp_err_t on bus error.
 */
esp_err_t lis3dhtr_check_who_am_i(uint8_t *value);

/**
 * @brief Read a single raw accelerometer sample (X, Y, Z).
 *
 * Reads 6 consecutive bytes starting at OUT_X_L using the multi-read
 * auto-increment address (bit 7 of register address set high).
 *
 * @param[out] accel  Pointer to structure to receive raw counts
 * @return ESP_OK on success, or esp_err_t on I2C failure.
 */
esp_err_t lis3dhtr_read_accel(lis3dhtr_accel_t *accel);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DHTR_H */
