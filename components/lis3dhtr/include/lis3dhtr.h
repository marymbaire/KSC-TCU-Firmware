/**
 * @file lis3dhtr.h
 * @brief LIS3DHTR Accelerometer Driver for KSC TCU V1.1 (ESP32-S3, I2C)
 *
 * Provides full accelerometer functionality relevant to a three-wheeled
 * electric motorcycle (trike):
 *
 *   - Motion detection (vehicle moving vs. stationary)
 *   - Tilt / roll / pitch angle estimation
 *   - Impact / shock event detection via hardware interrupt on INT1
 *   - Free-fall detection (e.g. trike tipped over)
 *   - Continuous raw acceleration sampling
 *
 * Hardware wiring (TCU V1.1 board):
 *   SCL  → GPIO47  (4.7 kΩ external pull-up to 3.3 V)
 *   SDA  → GPIO48  (4.7 kΩ external pull-up to 3.3 V)
 *   SA0  → GND     → I2C address = 0x18
 *   CS   → 3.3 V via 4.7 kΩ (I2C mode selected)
 *   INT1 → GPIO7   (hardware interrupt — click and activity detection)
 *   INT2 → not connected
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef LIS3DHTR_H_
#define LIS3DHTR_H_

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * DEVICE CONSTANTS
 * ========================================================================= */

/** I2C address with SA0 tied to GND */
#define LIS3DHTR_I2C_ADDR           0x18

/** Expected value of WHO_AM_I register — confirms device identity */
#define LIS3DHTR_WHO_AM_I_VAL       0x33

/* =========================================================================
 * REGISTER MAP (all registers used by this driver)
 * ========================================================================= */

#define LIS3DHTR_REG_WHO_AM_I       0x0F
#define LIS3DHTR_REG_CTRL_REG1      0x20  /**< ODR, power mode, axes        */
#define LIS3DHTR_REG_CTRL_REG2      0x21  /**< High-pass filter             */
#define LIS3DHTR_REG_CTRL_REG3      0x22  /**< INT1 pin function selection  */
#define LIS3DHTR_REG_CTRL_REG4      0x23  /**< Full-scale, HR mode, BDU     */
#define LIS3DHTR_REG_CTRL_REG5      0x24  /**< FIFO enable, latch INT       */
#define LIS3DHTR_REG_CTRL_REG6      0x25  /**< INT2 pad control             */
#define LIS3DHTR_REG_REFERENCE      0x26  /**< Reference/DC offset          */
#define LIS3DHTR_REG_STATUS_REG     0x27  /**< Data-ready flags             */
#define LIS3DHTR_REG_OUT_X_L        0x28  /**< X low byte (multi-read base) */
#define LIS3DHTR_REG_OUT_X_H        0x29
#define LIS3DHTR_REG_OUT_Y_L        0x2A
#define LIS3DHTR_REG_OUT_Y_H        0x2B
#define LIS3DHTR_REG_OUT_Z_L        0x2C
#define LIS3DHTR_REG_OUT_Z_H        0x2D
#define LIS3DHTR_REG_INT1_CFG       0x30  /**< INT1 axis/dir enable         */
#define LIS3DHTR_REG_INT1_SRC       0x31  /**< INT1 source (read to clear)  */
#define LIS3DHTR_REG_INT1_THS       0x32  /**< INT1 threshold               */
#define LIS3DHTR_REG_INT1_DURATION  0x33  /**< INT1 minimum duration        */
#define LIS3DHTR_REG_CLICK_CFG      0x38  /**< Click axis enable            */
#define LIS3DHTR_REG_CLICK_SRC      0x39  /**< Click source (read to clear) */
#define LIS3DHTR_REG_CLICK_THS      0x3A  /**< Click threshold              */
#define LIS3DHTR_REG_TIME_LIMIT     0x3B  /**< Click time limit             */
#define LIS3DHTR_REG_TIME_LATENCY   0x3C  /**< Double-click latency         */
#define LIS3DHTR_REG_TIME_WINDOW    0x3D  /**< Double-click window          */

/* =========================================================================
 * CTRL_REG1 bit values
 * ========================================================================= */

/** 100 Hz ODR, normal mode, X/Y/Z enabled: 0x57 = 0101_0111 */
#define LIS3DHTR_CTRL1_100HZ_NORMAL  0x57

/** 25 Hz ODR, normal mode, X/Y/Z enabled: 0x37 = 0011_0111
 *  Sufficient for motion/tilt on a vehicle; saves ~75% current vs 100 Hz */
#define LIS3DHTR_CTRL1_25HZ_NORMAL   0x37

/** 200 Hz ODR, normal power, X/Y/Z enabled: 0x67 = 0110_0111 */
#define LIS3DHTR_CTRL1_200HZ_NORMAL  0x67

/*
 * INT1_THS: Motion detection threshold
 *
 * Per LIS3DH datasheet Table 59:
 *   1 LSB = 16 mg  @ FS = ±2g
 *   1 LSB = 32 mg  @ FS = ±4g
 *   1 LSB = 62 mg  @ FS = ±8g
 *   1 LSB = 186 mg @ FS = ±16g   ← active
 *
 * Register: 8 bits, bit7=0 (unused), THS[6:0] = 7-bit value (0–127).
 *
 * With HP_IA1=1 (HPF on INT1), gravity is removed before comparison.
 * Threshold represents pure dynamic acceleration.
 *
 * At ±16g, 1 LSB = 186 mg:
 *   5  LSB =  930 mg ≈ 0.9g  (gentle motion, risk of vibration false trigger)
 *   8  LSB = 1488 mg ≈ 1.5g  (recommended — detects clear vehicle movement)
 *   16 LSB = 2976 mg ≈ 3.0g  (only strong acceleration triggers)
 *   26 LSB = 4836 mg ≈ 4.8g  (previous value — far too high for motion detect)
 *
 * Recommendation: use 8 LSB (1.5g) for trike motion detection.
 */
#define LIS3DHTR_MOTION_THS_DEFAULT  8    /* 8 × 186 mg = 1488 mg ≈ 1.5g  */

/*
 * INT1_DURATION is an 8-bit register (values 0-127).
 * At 200 Hz ODR: 1 measurement cycle = 1/200 s = 5 ms per LSB.
 * Required debounce: 200 ms / 5 ms per LSB = 40 LSB.
 */
#define LIS3DHTR_MOTION_DUR_DEFAULT  40

/* =========================================================================
 * IMPACT DETECTION THRESHOLDS — all full-scale ranges at 200 Hz ODR
 * =========================================================================
 * CLICK_THS register: 7-bit value, 1 LSB = FS/128.
 * LIR_Click (bit 7) = 1 to latch the interrupt.
 * TIME_LIMIT: max click pulse duration in ODR steps (1 step = 5 ms at 200 Hz).
 *
 * Automotive impact target: ≥5 g instantaneous shock (crash/severe pothole).
 *
 *  FS=±2g : 1 LSB=15.625mg, 5g=320LSB → exceeds 7-bit max(127)=1.98g.
 *            At ±2g the max settable threshold is 127 LSB (~2g). A true
 *            5g crash saturates the ADC so impact is still detected.
 *  FS=±4g : 1 LSB=31.25mg,  5g=160LSB → exceeds max. Use 127=~3.97g.
 *  FS=±8g : 1 LSB=62.5mg,   5g= 80LSB ✓  Use 80.
 *  FS=±16g: 1 LSB=125mg,    5g= 40LSB ✓  Use 40.  ← ACTIVE (16g mode)
 *
 * TIME_LIMIT=2 → 10 ms max pulse duration. Genuine impacts are short
 * transients; road vibration sustains longer and won't false-trigger.
 * ========================================================================= */

#define LIS3DHTR_IMPACT_THS_2G    127   /**< ~1.98 g (max at ±2g)           */
#define LIS3DHTR_IMPACT_THS_4G    127   /**< ~3.97 g (max at ±4g)           */
#define LIS3DHTR_IMPACT_THS_8G     80   /**< ~5.0 g  at ±8g                 */
#define LIS3DHTR_IMPACT_THS_16G    27   /**< ~5.0 g  at ±16g  ← current     */
#define LIS3DHTR_IMPACT_TIME_LIMIT  2   /**< 2 × 5 ms = 10 ms max pulse     */

/* =========================================================================
 * CTRL_REG4 values for each full-scale range (HR=1, BDU=1)
 * ========================================================================= */
#define LIS3DHTR_CTRL4_2G    0x88  /**< BDU=1, FS=±2g,  HR=1              */
#define LIS3DHTR_CTRL4_4G    0x98  /**< BDU=1, FS=±4g,  HR=1              */
#define LIS3DHTR_CTRL4_8G    0xA8  /**< BDU=1, FS=±8g,  HR=1              */
#define LIS3DHTR_CTRL4_16G   0xB8  /**< BDU=1, FS=±16g, HR=1              */

/* =========================================================================
 * SENSITIVITY per LSB after >>4 (12-bit HR mode)
 * ========================================================================= */
#define LIS3DHTR_SENS_MG_2G    1.0f   /**< mg/LSB at ±2g  HR               */
#define LIS3DHTR_SENS_MG_4G    2.0f   /**< mg/LSB at ±4g  HR               */
#define LIS3DHTR_SENS_MG_8G    4.0f   /**< mg/LSB at ±8g  HR               */
#define LIS3DHTR_SENS_MG_16G  12.0f /**< mg/LSB at ±16g HR (16000/1365)  */
/* =========================================================================
 * DATA STRUCTURES
 * ========================================================================= */

/**
 * @brief Raw 16-bit acceleration counts from all three axes.
 */
typedef struct {
    int16_t x;   /**< Raw X-axis counts (2's complement) */
    int16_t y;   /**< Raw Y-axis counts                  */
    int16_t z;   /**< Raw Z-axis counts                  */
} lis3dhtr_raw_t;

/**
 * @brief Calibrated acceleration values in milli-g.
 *
 * At ±2 g full-scale with 16-bit output (high-resolution mode):
 *   sensitivity = 1 mg/digit
 */
typedef struct {
    float x_mg;  /**< X acceleration (mg) */
    float y_mg;  /**< Y acceleration (mg) */
    float z_mg;  /**< Z acceleration (mg) */
} lis3dhtr_accel_mg_t;

/**
 * @brief Derived trike orientation and motion state.
 */
typedef struct {
    float    roll_deg;          /**< Roll angle — retained, not published    */
    float    pitch_deg;         /**< Pitch angle — retained, not published   */
    float    accel_mag_mg;      /**< Total vector magnitude (mg)             */
    float    accel_rms_mg;      /**< RMS of X²+Y²+Z² — for vibration        */
    float    x_mg;              /**< X-axis acceleration (mg) — published    */
    float    y_mg;              /**< Y-axis acceleration (mg) — published    */
    float    z_mg;              /**< Z-axis acceleration (mg) — published    */
    bool     in_motion;			/**< true if vehicle is moving (HW interrupt set)*/
    bool     impact_detected;	/**< true if a shock/click event was latched      */
    bool     free_fall;			/**< true if free-fall condition detected         */
} lis3dhtr_motion_t;

/* =========================================================================
 * OPERATING MODE — switchable between production (HPF, ±2g) and test
 * (no HPF, ±16g, gravity visible) per req 4
 * ========================================================================= */

typedef enum {
    LIS3DHTR_MODE_PRODUCTION = 0,  /**< HPF on, ±2g, vibration measurement  */
    LIS3DHTR_MODE_TEST       = 1,  /**< HPF off, ±16g, gravity component visible */
} lis3dhtr_mode_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise the I2C bus and configure the LIS3DHTR.
 *
 * Installs the I2C master driver, verifies WHO_AM_I, configures ODR,
 * full-scale range, high-resolution mode, and sets up the hardware
 * interrupt on INT1 for activity / inactivity detection.
 *
 * @return ESP_OK on success.
 *         ESP_ERR_NOT_FOUND   if WHO_AM_I does not match 0x33.
 *         ESP_ERR_INVALID_STATE if I2C driver installation fails.
 */
esp_err_t lis3dhtr_init(void);

/**
 * @brief Read raw 16-bit acceleration counts from all three axes.
 *
 * @param[out] raw  Pointer to receive raw counts.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_read_raw(lis3dhtr_raw_t *raw);

/**
 * @brief Read calibrated acceleration in milli-g from all three axes.
 *
 * Converts raw counts using the ±2 g sensitivity constant.
 *
 * @param[out] accel  Pointer to receive mg values.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_read_accel_mg(lis3dhtr_accel_mg_t *accel);

/**
 * @brief Read derived trike motion state.
 *
 * Computes roll, pitch, total acceleration magnitude, and checks the
 * INT1_SRC register to determine if the hardware motion-detection
 * interrupt has fired since the last call.
 *
 * @param[out] motion  Pointer to receive motion state.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_read_motion(lis3dhtr_motion_t *motion);

/**
 * @brief Check whether the vehicle is currently in motion.
 *
 * Uses the hardware activity-detection interrupt result latched in
 * INT1_SRC.  Reading INT1_SRC clears the latch automatically.
 *
 * @return true if motion is detected, false if stationary.
 */
bool lis3dhtr_is_in_motion(void);

/**
 * @brief Check and clear any latched impact event.
 *
 * @return true if an impact was detected since the last call.
 */
bool lis3dhtr_check_clear_impact(void);

/**
 * @brief Read WHO_AM_I register and verify the expected value 0x33.
 *
 * @param[out] value  Byte read from the register.
 * @return ESP_OK if read and value == 0x33.
 *         ESP_ERR_INVALID_RESPONSE if value is wrong.
 */
esp_err_t lis3dhtr_check_who_am_i(uint8_t *value);

/**
 * @brief Reconfigure the accelerometer operating mode at runtime.
 *
 * PRODUCTION mode: HPF on (CTRL_REG2=0x39), ±2g, 200 Hz. Output
 * registers show zero when stationary; only vibration is measured.
 * Impact detection: CLICK_THS threshold for ±16g.
 *
 * TEST mode: HPF off (CTRL_REG2=0x00), ±16g, 200 Hz. Gravity
 * component is visible (Z≈1000mg when flat). Useful for verifying
 * sensor orientation and confirming 1g component on each axis.
 * Impact detection: CLICK_THS threshold for ±16g.
 *
 * Both modes retain impact detection (CLICK_CFG) and motion detection
 * (INT1_CFG) interrupt configuration.
 *
 * @param[in] mode  LIS3DHTR_MODE_PRODUCTION or LIS3DHTR_MODE_TEST.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_configure_mode(lis3dhtr_mode_t mode);

/**
 * @brief Get the currently active operating mode.
 * @return Current lis3dhtr_mode_t.
 */
lis3dhtr_mode_t lis3dhtr_get_mode(void);

/**
 * @brief Scan the I2C bus and log all responding addresses.
 *
 * Useful during factory test to confirm device is wired correctly.
 */
void lis3dhtr_i2c_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DHTR_H_ */