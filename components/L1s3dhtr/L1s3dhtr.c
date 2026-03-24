/**
 * @file lis3dhtr.c
 * @brief Minimal LIS3DHTR accelerometer driver implementation (ESP32-S3, I2C)
 *
 * See lis3dhtr.h for full hardware wiring notes and API documentation.
 */

#include "L1s3dhtr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>

static const char *TAG = "LIS3DHTR";

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

/**
 * @brief Write a single byte to a LIS3DHTR register.
 *
 * @param reg   Register address
 * @param value Byte to write
 * @return esp_err_t
 */
static esp_err_t lis3dhtr_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(LIS3DHTR_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read one or more bytes from the LIS3DHTR starting at reg.
 *
 * For multi-byte reads the register address is OR'd with 0x80 to enable
 * the LIS3DHTR internal auto-increment (SUB bit 7 = 1).
 *
 * @param reg   Starting register address
 * @param dst   Buffer to receive bytes
 * @param len   Number of bytes to read
 * @return esp_err_t
 */
static esp_err_t lis3dhtr_read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    /* Set SUB bit 7 for auto-increment when reading multiple registers */
    uint8_t reg_addr = (len > 1) ? (reg | 0x80) : reg;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Write phase: send register address */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    /* Read phase: repeated start then read bytes */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, dst, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dst + (len - 1), I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(LIS3DHTR_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t lis3dhtr_init(void)
{
    ESP_LOGI(TAG, "Initialising I2C bus — SCL:IO%d  SDA:IO%d  Addr:0x%02X",
             LIS3DHTR_SCL_PIN, LIS3DHTR_SDA_PIN, LIS3DHTR_I2C_ADDR);

    /* Install I2C master driver */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = LIS3DHTR_SDA_PIN,
        .scl_io_num       = LIS3DHTR_SCL_PIN,
        /*
         * External 4.7 kΩ pull-ups are fitted on the PCB.
         * Disable internal pull-ups to avoid fighting the external ones.
         */
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = LIS3DHTR_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(LIS3DHTR_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(LIS3DHTR_I2C_PORT,
                              I2C_MODE_MASTER,
                              0, 0,   // rx/tx buffer sizes (not used in master)
                              0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C driver installed at %d Hz", LIS3DHTR_I2C_FREQ_HZ);

    /* Verify device identity */
    uint8_t who_am_i = 0;
    ret = lis3dhtr_check_who_am_i(&who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I check failed — is the device wired correctly?");
        return ret;
    }

    /*
     * Configure CTRL_REG1:
     *   ODR = 100 Hz, normal power mode, X/Y/Z axes enabled.
     *   0x57 = 0101 0111b
     */
    ret = lis3dhtr_write_reg(LIS3DHTR_REG_CTRL_REG1, LIS3DHTR_CTRL_REG1_100HZ_NORMAL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL_REG1: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * Configure CTRL_REG4:
     *   Full-scale ±2g, high-resolution mode, block data update disabled.
     *   0x08 = BDU=0, BLE=0, FS=00 (±2g), HR=1, ST=00, SIM=0
     */
    ret = lis3dhtr_write_reg(LIS3DHTR_REG_CTRL_REG4, 0x08);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL_REG4: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LIS3DHTR configured — 100 Hz, ±2g, high-res, all axes ON");
    return ESP_OK;
}

void lis3dhtr_i2c_scan(void)
{
    ESP_LOGI(TAG, "--- I2C Bus Scan (port %d) ---", LIS3DHTR_I2C_PORT);

    int found = 0;

    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(LIS3DHTR_I2C_PORT, cmd,
                                              pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X%s",
                     addr,
                     (addr == LIS3DHTR_I2C_ADDR) ? "  ← LIS3DHTR (expected)" : "");
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "  No I2C devices found! Check wiring and pull-ups.");
    } else {
        ESP_LOGI(TAG, "  Scan complete — %d device(s) found.", found);
    }

    ESP_LOGI(TAG, "--- End I2C Scan ---");
}

esp_err_t lis3dhtr_check_who_am_i(uint8_t *value)
{
    if (value == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = lis3dhtr_read_regs(LIS3DHTR_REG_WHO_AM_I, value, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X) — %s",
             *value,
             LIS3DHTR_WHO_AM_I_VAL,
             (*value == LIS3DHTR_WHO_AM_I_VAL) ? "PASS ✓" : "FAIL ✗");

    if (*value != LIS3DHTR_WHO_AM_I_VAL) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t lis3dhtr_read_accel(lis3dhtr_accel_t *accel)
{
    if (accel == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6] = {0};

    /* OUT_X_L = 0x28; with auto-increment bit (0x80) → 0xA8 */
    esp_err_t ret = lis3dhtr_read_regs(LIS3DHTR_REG_OUT_X_L, raw, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accel data: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * LIS3DHTR outputs data in little-endian format (LSB first).
     * Combine low and high bytes into signed 16-bit values.
     */
    accel->x = (int16_t)((raw[1] << 8) | raw[0]);
    accel->y = (int16_t)((raw[3] << 8) | raw[2]);
    accel->z = (int16_t)((raw[5] << 8) | raw[4]);

    return ESP_OK;
}