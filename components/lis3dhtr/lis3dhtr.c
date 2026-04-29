/**
 * @file lis3dhtr.c
 * @brief LIS3DHTR Accelerometer Driver Implementation for KSC TCU V1.1
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "lis3dhtr.h"
#include "board_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

#define TAG "LIS3DHTR"

/* =========================================================================
 * SENSITIVITY CONSTANT
 * =========================================================================
 * At ±2 g full-scale in high-resolution (16-bit) mode:
 *   sensitivity = 1 mg / digit
 *
 * Output is left-justified in 16-bit register; actual resolution is
 * 12-bit (bits [15:4]).  Right-shift 4 then multiply by 1.0 mg/digit.
 * ========================================================================= */

/* Active sensitivity — updated when mode changes */
static float s_sensitivity_mg = LIS3DHTR_SENS_MG_16G;
static lis3dhtr_mode_t s_mode = LIS3DHTR_MODE_PRODUCTION;

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static bool s_initialized = false;

/* =========================================================================
 * LOW-LEVEL I2C PRIMITIVES
 * ========================================================================= */

/**
 * @brief Write a single byte to a register.
 *
 * @param[in] reg    Register address.
 * @param[in] value  Byte to write.
 * @return esp_err_t
 */
static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read one or more consecutive bytes starting at reg.
 *
 * Sets bit 7 of the register address to enable the LIS3DHTR
 * auto-increment (SUB bit 7 = 1) for multi-byte reads.
 *
 * @param[in]  reg  Starting register address.
 * @param[out] dst  Buffer to receive bytes.
 * @param[in]  len  Number of bytes to read.
 * @return esp_err_t
 */
static esp_err_t read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    uint8_t reg_addr = (len > 1) ? (reg | 0x80) : reg;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);   /* Repeated start */
    i2c_master_write_byte(cmd,
        (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, dst, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dst + (len - 1), I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

esp_err_t lis3dhtr_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* Install I2C master driver (uses board_config.h pin definitions) */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BOARD_I2C_SDA_PIN,
        .scl_io_num       = BOARD_I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,  /* External pull-ups fitted */
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = BOARD_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(BOARD_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(BOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify device identity */
    uint8_t who = 0;
    ret = lis3dhtr_check_who_am_i(&who);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I check failed — check wiring");
        return ret;
    }

    /*
     * CTRL_REG1: 200 Hz ODR, normal power, X/Y/Z enabled
     *   0x67 = 0011_0111
     *   [7:4] ODR = 0110 → 200 Hz
     *   [3]   LPen = 0   → Normal mode
     *   [2:0] Zen/Yen/Xen = 111
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG1, LIS3DHTR_CTRL1_200HZ_NORMAL);
    if (ret != ESP_OK) { goto init_fail; }
    
	/* 
	 * CTRL_REG2: Enable high-pass filter on output data registers and INT1 output
	 * CTRL_REG2 = 0x39 = 0011 1001
	 * Bit 7 = 0  → HPM1    = 0  ┐  HPF mode = Normal
	 * Bit 6 = 0  → HPM0    = 0  ┘
	 * Bit 5 = 1  → HPCF2   = 1  ┐  Cutoff = 1 Hz at 200 Hz ODR
	 * Bit 4 = 1  → HPCF1   = 1  ┘
	 * Bit 3 = 1  → FDS     = 1  ← HPF filtered data to OUT registers
	 * Bit 2 = 0  → HPCLICK = 0  ← HPF NOT applied to click (raw data for impact)
	 * Bit 1 = 0  → HP_IA2  = 0  ← HPF NOT applied to INT2
	 * Bit 0 = 1  → HP_IA1  = 1  ← HPF applied to INT1 (gravity-free motion detect)
	 */
	ret = write_reg(LIS3DHTR_REG_CTRL_REG2, 0x39);     
	if (ret != ESP_OK) { goto init_fail; }
    /*
     * CTRL_REG4: ±16 g, high-resolution mode, BDU enabled
     *   0xB8 = 1011_1000
     *   [7]   BDU = 1   → Block data update (avoids reading high/low
     *                       byte from two different samples)
     *   [6]   BLE = 0   → Little-endian (LSB at lower address)
     *   [5:4] FS  = 11  → ±16 g full-scale
     *   [3]   HR  = 1   → High-resolution (12-bit effective)
     *   [1:0] SIM = 00  → SPI 4-wire (irrelevant in I2C mode)
     * 	 ±16g chosen to prevent saturation during automotive impacts.
     * 	 Resolution is 11.718 mg/LSB — sufficient for vibration at 200 Hz.
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG4, LIS3DHTR_CTRL4_16G);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * CTRL_REG5: Latch interrupt on INT1
     *   0x08 = 0000_1000
     *   [3] LIR_INT1 = 1 → Interrupt latched until INT1_SRC is read
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG5, 0x08);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * CTRL_REG3: Route both activity interrupt (IA1) AND click interrupt
     * to INT1 pin.
     *   0xC0 = 1100_0000
     *   [7] I1_CLICK = 1 → Click interrupt routed to INT1
     *   [6] I1_IA1   = 1 → Activity (INT1_CFG) routed to INT1
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG3, 0xC0);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * INT1_CFG: Enable OR combination of high events on all axes
     *   0x2A = 0010_1010
     *   [5] ZHIE = 1, [3] YHIE = 1, [1] XHIE = 1
     *   Triggers when any axis exceeds the threshold
     */
    ret = write_reg(LIS3DHTR_REG_INT1_CFG, 0x2A);
    if (ret != ESP_OK) { goto init_fail; }   

    /*
     * INT1_THS: Motion detection threshold
     * With HPF on and ±16g: 1 LSB = 125 mg.
     * LIS3DHTR_MOTION_THS_DEFAULT = 26 LSB × 125 mg = 3250 mg ≈ 3.25 g
     *
     * Note: INT1_THS uses the same FS/128 scale as CLICK_THS.
     * At ±16g: 1 LSB = 16000/128 = 125 mg.
     * 26 × 125 = 3250 mg → triggers on sustained motion above 3.25g.
     * This is appropriate for motion detection (trike moving) vs impact
     * (short transient handled by CLICK). For very gentle trike movement
     * consider reducing to 8–12 LSB (~1–1.5g).
     */
    ret = write_reg(LIS3DHTR_REG_INT1_THS, LIS3DHTR_MOTION_THS_DEFAULT);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * INT1_DURATION: Minimum event duration before interrupt fires
     * At 25 Hz, 1 LSB = 1/25 = 40 ms
     * At 200Hz, 1 LSB = 1/200 = 5 ms
     * INT1_DURATION: 40 LSB × 5 ms = 200 ms debounce at 200 Hz
     */
    ret = write_reg(LIS3DHTR_REG_INT1_DURATION, LIS3DHTR_MOTION_DUR_DEFAULT);
    if (ret != ESP_OK) { goto init_fail; }
    
    /* -----------------------------------------------------------------------
     * CLICK / IMPACT DETECTION
     * -----------------------------------------------------------------------
     * CLICK_CFG (0x38): Enable single-click on all three axes.
     *   0x15 = 0001_0101
     *   [4] XS = 1 → single-click X enabled
     *   [2] YS = 1 → single-click Y enabled
     *   [0] ZS = 1 → single-click Z enabled
     *   Double-click bits [5,3,1] = 0 (not used — impact is single event)
     */
    ret = write_reg(LIS3DHTR_REG_CLICK_CFG, 0x15);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * CLICK_THS (0x3A): Impact threshold + latch enable
     *   Bit 7 (LIR_Click) = 1 → latch interrupt until CLICK_SRC is read
     *   Bits [6:0]         = threshold in FS/128 units
     *
     * At ±16g: 1 LSB = 186 mg. 27 LSB = 5000 mg = 5g (automotive impact).
     *   0x80 | 40 = 0xA8
     */
    ret = write_reg(LIS3DHTR_REG_CLICK_THS,
                    0x80 | LIS3DHTR_IMPACT_THS_16G);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * TIME_LIMIT (0x3B): Maximum click pulse duration
     *   At 200 Hz, 1 LSB = 5 ms.
     *   2 LSB = 10 ms → genuine impact transient window.
     *   Road bumps that last >10 ms will not trigger click interrupt.
     */
    ret = write_reg(LIS3DHTR_REG_TIME_LIMIT, LIS3DHTR_IMPACT_TIME_LIMIT);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * TIME_LATENCY (0x3C): For single-click only, set to 0
     * (latency only applies to double-click detection)
     */
    ret = write_reg(LIS3DHTR_REG_TIME_LATENCY, 0x00);
    if (ret != ESP_OK) { goto init_fail; }    

    /*
     * Configure INT1 GPIO as input (no pull — hardware driver on board)
     */
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << BOARD_ACCEL_INT1_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,  /* Polled — no FreeRTOS ISR needed */
    };
    gpio_config(&int_conf);

	s_mode = LIS3DHTR_MODE_PRODUCTION;
    s_initialized = true;
    ESP_LOGI(TAG, "LIS3DHTR initialised — 25 Hz, ±2 g, HR mode, INT1=IO%d",
             BOARD_ACCEL_INT1_PIN);
             
   vTaskDelay(pdMS_TO_TICKS(50));   // 50ms > 35ms required turn-on time at 200Hz         
    return ESP_OK;

init_fail:
    ESP_LOGE(TAG, "LIS3DHTR configuration failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t lis3dhtr_check_who_am_i(uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = read_regs(LIS3DHTR_REG_WHO_AM_I, value, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X) — %s",
             *value, LIS3DHTR_WHO_AM_I_VAL,
             (*value == LIS3DHTR_WHO_AM_I_VAL) ? "PASS" : "FAIL");

    return (*value == LIS3DHTR_WHO_AM_I_VAL) ? ESP_OK :
                                                ESP_ERR_INVALID_RESPONSE;
}

esp_err_t lis3dhtr_read_raw(lis3dhtr_raw_t *raw)
{
    if (!s_initialized || raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[6] = {0};
    esp_err_t ret = read_regs(LIS3DHTR_REG_OUT_X_L, buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acceleration data: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    /* LIS3DHTR outputs little-endian (LSB at lower address) */
    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    return ESP_OK;
}

esp_err_t lis3dhtr_read_accel_mg(lis3dhtr_accel_mg_t *accel)
{
    if (!s_initialized || accel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dhtr_raw_t raw;
    esp_err_t ret = lis3dhtr_read_raw(&raw);
    if (ret != ESP_OK) {
        return ret;
    }

    /*
     * In high-resolution 12-bit mode the output is left-justified.
     * Right-shift 4 bits to get the 12-bit value, then multiply by
     * sensitivity (1 mg / digit at ±2 g).
     */
	/* Use active sensitivity — changes with mode/FS setting */
	    accel->x_mg = (float)(raw.x >> 4) * s_sensitivity_mg;
	    accel->y_mg = (float)(raw.y >> 4) * s_sensitivity_mg;
	    accel->z_mg = (float)(raw.z >> 4) * s_sensitivity_mg;

    return ESP_OK;
}

esp_err_t lis3dhtr_read_motion(lis3dhtr_motion_t *motion)
{
    if (!s_initialized || motion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(motion, 0, sizeof(lis3dhtr_motion_t));

    /* Read calibrated acceleration */
    lis3dhtr_accel_mg_t accel;
    esp_err_t ret = lis3dhtr_read_accel_mg(&accel);
    if (ret != ESP_OK) {
        return ret;
    }
	/* Store raw axis values for publishing */
	motion->x_mg = accel.x_mg;
	motion->y_mg = accel.y_mg;
	motion->z_mg = accel.z_mg; 
	
	/* RMS acceleration — used by server for vibration analysis.
	 * RMS = sqrt((x² + y² + z²) / 3)
	 * Dividing by 3 normalises across axes; the server receives a
	 * single scalar that represents the overall vibration energy level.
	 * Note: accel_mag_mg = sqrt(x²+y²+z²) is the vector magnitude (not RMS).
	 * RMS and magnitude differ: magnitude includes the 1g gravity component
	 * on the dominant axis; RMS normalises it across three axes.             */
	motion->accel_rms_mg = sqrtf(
	    (accel.x_mg * accel.x_mg +
	     accel.y_mg * accel.y_mg +
	     accel.z_mg * accel.z_mg) / 3.0f);	   

    /* Total acceleration magnitude */
    motion->accel_mag_mg = sqrtf(accel.x_mg * accel.x_mg +
                                  accel.y_mg * accel.y_mg +
                                  accel.z_mg * accel.z_mg);

    /*
     * Roll angle: rotation around X-axis (lateral tilt of trike)
     *   roll = atan2(y, z)
     */
    motion->roll_deg  = atan2f(accel.y_mg, accel.z_mg) * (180.0f / M_PI);

    /*
     * Pitch angle: rotation around Y-axis (forward/backward tilt)
     *   pitch = atan2(-x, sqrt(y²+z²))
     */
    motion->pitch_deg = atan2f(-accel.x_mg,
                                sqrtf(accel.y_mg * accel.y_mg +
                                      accel.z_mg * accel.z_mg))
                        * (180.0f / M_PI);

    /*
     * Free-fall: acceleration magnitude drops well below 1 g.
     * Threshold: < 350 mg on all axes simultaneously.
     */
    motion->free_fall = (motion->accel_mag_mg < 350.0f);

    /*
     * Motion / impact: read INT1_SRC register.
     * Reading this register automatically clears the latched interrupt.
     * Bit 6 (IA) = 1 → at least one enabled event has occurred.
     * Bit 4/5 (ZHIE/ZLIE etc.) give axis detail.
     */
    uint8_t int1_src = 0;
    ret = read_regs(LIS3DHTR_REG_INT1_SRC, &int1_src, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    bool hw_event = (int1_src & 0x40) != 0;   /* IA bit */
    motion->in_motion = hw_event;

    /*
     * Impact detection: use the click/shock detection register.
     * Read CLICK_SRC to check if a single or double click occurred.
     */
    uint8_t click_src = 0;
    ret = read_regs(LIS3DHTR_REG_CLICK_SRC, &click_src, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    motion->impact_detected = (click_src & 0x40) != 0;  /* IA bit */

    ESP_LOGD(TAG, "Motion: roll=%.1f° pitch=%.1f° |a|=%.0f mg "
             "in_motion=%d impact=%d ff=%d",
             motion->roll_deg, motion->pitch_deg, motion->accel_mag_mg,
             motion->in_motion, motion->impact_detected, motion->free_fall);

    return ESP_OK;
}

bool lis3dhtr_is_in_motion(void)
{
    if (!s_initialized) {
        return false;
    }

    /*
     * Check the physical INT1 GPIO level as a fast path — the hardware
     * interrupt is asserted (active-high after latch) when activity is
     * detected.  This avoids an I2C transaction on every poll when the
     * trike is stationary.
     */
    int gpio_level = gpio_get_level(BOARD_ACCEL_INT1_PIN);
    if (gpio_level == 0) {
        return false;   /* No interrupt pending → not in motion */
    }

    /*
     * GPIO is high → read INT1_SRC to confirm and clear the latch.
     */
    uint8_t int1_src = 0;
    if (read_regs(LIS3DHTR_REG_INT1_SRC, &int1_src, 1) != ESP_OK) {
        return false;
    }
    return (int1_src & 0x40) != 0;
}

bool lis3dhtr_check_clear_impact(void)
{
    if (!s_initialized) {
        return false;
    }

    uint8_t click_src = 0;
    if (read_regs(LIS3DHTR_REG_CLICK_SRC, &click_src, 1) != ESP_OK) {
        return false;
    }
    return (click_src & 0x40) != 0;
}

esp_err_t lis3dhtr_configure_mode(lis3dhtr_mode_t mode)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    if (mode == LIS3DHTR_MODE_TEST) {
        /*
         * TEST MODE: Remove HPF so gravity component is visible.
         * Switch to ±16g full-scale.
         * Useful for verifying 1g component when tilting each axis.
         *
         * CTRL_REG2 = 0x00 — no HPF anywhere
         * CTRL_REG4 = LIS3DHTR_CTRL4_16G — ±16g, HR, BDU
         * CLICK_THS — ±16g impact threshold (5g)
         * INT1_THS  — motion threshold recalculated for ±16g
         */
        ret = write_reg(LIS3DHTR_REG_CTRL_REG2, 0x00);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CTRL_REG4, LIS3DHTR_CTRL4_16G);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CLICK_THS,
                        0x80 | LIS3DHTR_IMPACT_THS_16G);
        if (ret != ESP_OK) return ret;

        s_sensitivity_mg = LIS3DHTR_SENS_MG_16G;
        s_mode           = LIS3DHTR_MODE_TEST;

        ESP_LOGI(TAG, "Mode → TEST: HPF OFF, ±16g, gravity visible. "
                 "Expected Z≈+856mg flat (1000mg / 11.718mg/LSB × 10)");
    } else {
        /*
         * PRODUCTION MODE: HPF on, ±16g (retain 16g for impact range).
         * Output registers show ~0mg when stationary (DC removed).
         */
        ret = write_reg(LIS3DHTR_REG_CTRL_REG2, 0x39);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CTRL_REG4, LIS3DHTR_CTRL4_16G);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CLICK_THS,
                        0x80 | LIS3DHTR_IMPACT_THS_16G);
        if (ret != ESP_OK) return ret;

        s_sensitivity_mg = LIS3DHTR_SENS_MG_16G;
        s_mode           = LIS3DHTR_MODE_PRODUCTION;

        ESP_LOGI(TAG, "Mode → PRODUCTION: HPF ON, ±16g, vibration only");
    }

    return ESP_OK;
}

lis3dhtr_mode_t lis3dhtr_get_mode(void)
{
    return s_mode;
}

void lis3dhtr_i2c_scan(void)
{
    ESP_LOGI(TAG, "--- I2C Bus Scan (port %d) ---", BOARD_I2C_PORT);
    int found = 0;

    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd,
                                              pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Device at 0x%02X%s", addr,
                     (addr == LIS3DHTR_I2C_ADDR) ? " <- LIS3DHTR" : "");
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "  No devices found! Check wiring and pull-ups.");
    } else {
        ESP_LOGI(TAG, "  Scan complete — %d device(s) found.", found);
    }
    ESP_LOGI(TAG, "--- End I2C Scan ---");
}