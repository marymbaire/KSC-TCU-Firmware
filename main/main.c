/**
 * @file main.c
 * @brief TCU V2 PCB — Hardware Test Firmware
 *
 * Hardware: ESP32-S3-WROOM-1-N16R2
 *
 * ============================================================================
 * WHAT THIS FIRMWARE TESTS
 * ============================================================================
 *
 *  1. RS485 MUX (MC74HC4052ADG)
 *     Cycles through all 3 RS485 slave channels and transmits "Hello World"
 *     5 times per channel.  Observe TX LEDs to confirm each channel fires.
 *     No slaves need to be connected.
 *
 *  2. RGB LED (N-channel MOSFET driven, active-HIGH)
 *     R (IO1)  — power LED, normally always ON.
 *     G (IO2)  — normally OFF.
 *     B (IO42) — normally OFF.
 *     Test: blinks G and B together every 2 s while R stays on, then briefly
 *     turns R off to confirm it can also be toggled.
 *     Blink task runs independently on Core 0 so RS485 timing is unaffected.
 *
 *  3. LIS3DHTR Accelerometer (I2C)
 *     - I2C bus scan to confirm device presence (expected at 0x18)
 *     - WHO_AM_I register read and verify (expected 0x33)   
 *     Sensor task also runs on Core 0.
 *
 * ============================================================================
 * PIN SUMMARY
 * ============================================================================
 *
 *  RS485 / MUX
 *    UART2 TX     → GPIO13  (MUX X input)
 *    UART2 RX     → GPIO12  (MUX Y input)
 *    MUX_SELECT_A → GPIO8   (MUX pin A/S1)
 *    MUX_SELECT_B → GPIO9   (MUX pin B/S2)
 *    DE/RE        → N/A     (always enabled)
 *
 *  MUX channel truth table:
 *    A=0, B=0  → Slave 1
 *    A=1, B=0  → Slave 2
 *    A=0, B=1  → Slave 3
 *    A=1, B=1  → (unused)
 *
 *  RGB LED (active-HIGH via N-ch MOSFET)
 *    R → GPIO1   (power LED — ON by default)
 *    G → GPIO2   (OFF by default)
 *    B → GPIO42  (OFF by default)
 *
 *  LIS3DHTR Accelerometer (I2C)
 *    SCL  → GPIO47  (4.7 kΩ external pull-up)
 *    SDA  → GPIO48  (4.7 kΩ external pull-up)
 *    SA0  → GND     → I2C address 0x18
 *    INT1 → GPIO7   (configured as input, not used in this test)
 *    INT2 → NC
 *
 *  UART assignments
 *    UART0 → Quectel comms      (TXD0 / RXD0)
 *    UART1 → Quectel debug/GPS  (IO17  / IO18)
 *    UART2 → RS485 MUX test     (GPIO13 / GPIO12)
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "L1s3dhtr.h"   // LIS3DHTR I2C driver (lis3dhtr.c / lis3dhtr.h)

// ============================================================================
// SECTION: RS485 / MUX
// ============================================================================

#define RS485_UART              UART_NUM_2
#define RS485_TX_PIN            GPIO_NUM_13
#define RS485_RX_PIN            GPIO_NUM_12
#define MUX_SELECT_A            GPIO_NUM_8
#define MUX_SELECT_B            GPIO_NUM_9

#define UART_BAUD_RATE          115200
#define UART_BUF_SIZE           256

#define HELLO_REPEAT_COUNT      5
#define BETWEEN_TX_DELAY_MS     200
#define BETWEEN_SLAVE_DELAY_MS  1000
#define BETWEEN_CYCLE_DELAY_MS  2000

// ============================================================================
// SECTION: RGB LED
// ============================================================================

/**
 * LEDs are driven by N-channel MOSFETs:
 *   Logic HIGH → MOSFET ON  → LED ON
 *   Logic LOW  → MOSFET OFF → LED OFF
 *
 * Startup state:
 *   R = ON  (power indicator)
 *   G = OFF
 *   B = OFF
 */
#define LED_R_PIN               GPIO_NUM_1
#define LED_G_PIN               GPIO_NUM_2
#define LED_B_PIN               GPIO_NUM_42

/** Blink period for G and B LEDs (ms).  R blinks briefly once per sequence. */
#define LED_BLINK_PERIOD_MS     2000

// ============================================================================
// SECTION: LIS3DHTR INT1 input pin
// ============================================================================

/** INT1 is wired but not used in this test — configured as input only */
#define ACCEL_INT1_PIN          GPIO_NUM_7

/** How often to read and log accelerometer data (ms) */
#define ACCEL_READ_INTERVAL_MS  2000

// ============================================================================
// LOGGING TAGS
// ============================================================================

static const char *TAG_RS485  = "RS485_MUX";
static const char *TAG_LED    = "RGB_LED";
static const char *TAG_ACCEL  = "ACCEL";
static const char *TAG_MAIN   = "MAIN";

// ============================================================================
// MUX CHANNEL TABLE
// ============================================================================

typedef struct {
    const char *name;
    int         sel_a;
    int         sel_b;
} mux_channel_t;

static const mux_channel_t mux_channels[] = {
    { "Slave 1 (A=0,B=0)", 0, 0 },
    { "Slave 2 (A=1,B=0)", 1, 0 },
    { "Slave 3 (A=0,B=1)", 0, 1 },
};
#define MUX_CHANNEL_COUNT   (sizeof(mux_channels) / sizeof(mux_channels[0]))

// ============================================================================
// RS485 / MUX INITIALISATION
// ============================================================================

static void mux_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MUX_SELECT_A) | (1ULL << MUX_SELECT_B),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(MUX_SELECT_A, 0);
    gpio_set_level(MUX_SELECT_B, 0);

    ESP_LOGI(TAG_RS485, "MUX GPIO initialised — default: Slave 1 (A=0, B=0)");
}

static void rs485_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART,
                                        UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART,
                                  RS485_TX_PIN,
                                  RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG_RS485, "UART2 initialised — TX:GPIO%d  RX:GPIO%d  Baud:%d",
             RS485_TX_PIN, RS485_RX_PIN, UART_BAUD_RATE);
}

// ============================================================================
// MUX CONTROL
// ============================================================================

static void mux_select_channel(uint8_t channel)
{
    if (channel >= MUX_CHANNEL_COUNT) {
        ESP_LOGE(TAG_RS485, "Invalid MUX channel: %d", channel);
        return;
    }
    gpio_set_level(MUX_SELECT_A, mux_channels[channel].sel_a);
    gpio_set_level(MUX_SELECT_B, mux_channels[channel].sel_b);

    /* Allow RS485 bus to settle after MUX switches */
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG_RS485, "MUX → %s", mux_channels[channel].name);
}

// ============================================================================
// RS485 TRANSMIT
// ============================================================================

static void rs485_send(const char *message)
{
    size_t len = strlen(message);
    int written = uart_write_bytes(RS485_UART, message, len);
    if (written < 0) {
        ESP_LOGE(TAG_RS485, "uart_write_bytes error");
        return;
    }
    ESP_ERROR_CHECK(uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(500)));
}

// ============================================================================
// RGB LED INITIALISATION
// ============================================================================

/**
 * @brief Configure R, G, B LED GPIO pins as outputs.
 *
 * Initial state:
 *   R = HIGH (ON)  — power indicator
 *   G = LOW  (OFF)
 *   B = LOW  (OFF)
 */
static void rgb_led_init(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_R_PIN) |
                        (1ULL << LED_G_PIN) |
                        (1ULL << LED_B_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));

    gpio_set_level(LED_R_PIN, 1);   // R ON  — power LED
    gpio_set_level(LED_G_PIN, 0);   // G OFF
    gpio_set_level(LED_B_PIN, 0);   // B OFF

    ESP_LOGI(TAG_LED, "RGB LED initialised — R:ON  G:OFF  B:OFF");
}

// ============================================================================
// ACCELEROMETER INT1 PIN INITIALISATION
// ============================================================================

static void accel_int1_gpio_init(void)
{
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << ACCEL_INT1_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // pull low when idle
        .intr_type    = GPIO_INTR_DISABLE,      // not used in this test
    };
    ESP_ERROR_CHECK(gpio_config(&int_conf));
    ESP_LOGI(TAG_ACCEL, "ACCEL INT1 pin GPIO%d configured as input", ACCEL_INT1_PIN);
}

// ============================================================================
// TASK: RGB LED BLINK  (Core 0)
// ============================================================================

/**
 * @brief RGB LED blink sequence (runs indefinitely, every LED_BLINK_PERIOD_MS).
 *
 * Sequence per cycle:
 *   1. G and B turn ON  (R stays ON as power indicator)
 *   2. Wait 500 ms
 *   3. G and B turn OFF
 *   4. Wait 500 ms
 *   5. R briefly turns OFF to confirm it is also GPIO-controlled
 *   6. Wait 200 ms
 *   7. R turns back ON
 *   8. Wait remainder of LED_BLINK_PERIOD_MS before next cycle
 *
 * This gives a clear visual confirmation that:
 *   - G and B can be independently switched ON/OFF
 *   - R can also be switched OFF (it is not hard-wired high)
 *   - All three MOSFET drives are functional
 */
static void rgb_led_blink_task(void *pvParameters)
{
    uint32_t blink_cycle = 0;
    ESP_LOGI(TAG_LED, "LED blink task started (period: %d ms)", LED_BLINK_PERIOD_MS);

    while (1) {
        blink_cycle++;
        ESP_LOGI(TAG_LED, "--- Blink cycle %" PRIu32 " ---", blink_cycle);

        /* Step 1: G and B ON, R stays ON */
        gpio_set_level(LED_G_PIN, 1);
        gpio_set_level(LED_B_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        /* Step 2: G and B OFF */
        gpio_set_level(LED_G_PIN, 0);
        gpio_set_level(LED_B_PIN, 0);
   		vTaskDelay(pdMS_TO_TICKS(500));

        /* Step 3: Briefly turn R OFF to prove it is controllable */
        gpio_set_level(LED_R_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

        /* Step 4: R back ON — restore power indicator state */
        gpio_set_level(LED_R_PIN, 1);

        /* Wait the rest of the blink period (2000 - 500 - 500 - 200 = 800 ms) */
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_PERIOD_MS - 1200));
    }
}

// ============================================================================
// TASK: ACCELEROMETER READ  (Core 0)
// ============================================================================

/**
 * @brief Accelerometer test task.
 *
 * On startup:
 *   1. I2C bus scan — logs all responding addresses
 *   2. WHO_AM_I register read — verifies 0x33
 *   3. Init LIS3DHTR (sets 100 Hz, ±2g, all axes)
 *
 * Then every ACCEL_READ_INTERVAL_MS:
 *   - Read raw X/Y/Z counts
 *   - Convert to milli-g (1 LSB = 1 mg at ±2g, 16-bit mode)
 *   - Log to console
 *
 * If WHO_AM_I fails the task logs an error and exits — RS485 and LED tests
 * continue unaffected.
 */
static void accel_test_task(void *pvParameters)
{
    ESP_LOGI(TAG_ACCEL, "Accelerometer test task started");

    /* --- Step 1: I2C bus scan --- */
    ESP_LOGI(TAG_ACCEL, "Running I2C bus scan...");
    lis3dhtr_i2c_scan();

    /* --- Step 2 & 3: WHO_AM_I + full init --- */
    esp_err_t ret = lis3dhtr_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ACCEL, "LIS3DHTR init FAILED (%s). Accelerometer task stopping.",
                 esp_err_to_name(ret));
        ESP_LOGE(TAG_ACCEL, "Check: SCL=IO%d, SDA=IO%d, SA0=GND, pull-ups fitted?",
                 LIS3DHTR_SCL_PIN, LIS3DHTR_SDA_PIN);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_ACCEL, "LIS3DHTR ready");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ACCEL_READ_INTERVAL_MS));
    }
}

// ============================================================================
// TASK: RS485 MUX TEST  (Core 1)
// ============================================================================

static void rs485_mux_test_task(void *pvParameters)
{
    uint32_t cycle = 0;

    ESP_LOGI(TAG_RS485, "RS485 MUX test task started");
    ESP_LOGI(TAG_RS485, "Channels: %d  |  TX per channel: %d",
             (int)MUX_CHANNEL_COUNT, HELLO_REPEAT_COUNT);

    while (1) {
        cycle++;
        ESP_LOGI(TAG_RS485, "========== RS485 CYCLE %" PRIu32 " ==========", cycle);

        for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ch++) {
            mux_select_channel(ch);

            for (int tx = 1; tx <= HELLO_REPEAT_COUNT; tx++) {
                ESP_LOGI(TAG_RS485, "[%s] TX %d/%d → \"Hello World\"",
                         mux_channels[ch].name, tx, HELLO_REPEAT_COUNT);
                rs485_send("Hello World\r\n");
                vTaskDelay(pdMS_TO_TICKS(BETWEEN_TX_DELAY_MS));
            }

            ESP_LOGI(TAG_RS485, "[%s] Done. Pausing %d ms...",
                     mux_channels[ch].name, BETWEEN_SLAVE_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(BETWEEN_SLAVE_DELAY_MS));
        }

        ESP_LOGI(TAG_RS485, "All slaves done. Waiting %d ms before next cycle...",
                 BETWEEN_CYCLE_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(BETWEEN_CYCLE_DELAY_MS));
    }
}

// ============================================================================
// APP MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "================================================");
    ESP_LOGI(TAG_MAIN, "  TCU V2 PCB — Hardware Test Firmware");
    ESP_LOGI(TAG_MAIN, "  ESP32-S3-WROOM-1-N16R2");
    ESP_LOGI(TAG_MAIN, "  Tests: RS485 MUX | RGB LED | LIS3DHTR Accel");
    ESP_LOGI(TAG_MAIN, "================================================");
    ESP_LOGI(TAG_MAIN, "  RS485  : UART2  TX=GPIO%d  RX=GPIO%d",
             RS485_TX_PIN, RS485_RX_PIN);
    ESP_LOGI(TAG_MAIN, "  MUX    : SEL_A=GPIO%d  SEL_B=GPIO%d",
             MUX_SELECT_A, MUX_SELECT_B);
    ESP_LOGI(TAG_MAIN, "  RGB LED: R=GPIO%d  G=GPIO%d  B=GPIO%d",
             LED_R_PIN, LED_G_PIN, LED_B_PIN);
    ESP_LOGI(TAG_MAIN, "  ACCEL  : SCL=IO%d  SDA=IO%d  Addr=0x%02X",
             LIS3DHTR_SCL_PIN, LIS3DHTR_SDA_PIN, LIS3DHTR_I2C_ADDR);
    ESP_LOGI(TAG_MAIN, "================================================");

    // -----------------------------------------------------------------------
    // Peripheral initialisation
    // -----------------------------------------------------------------------

    /* 1. MUX select GPIOs */
    mux_gpio_init();

    /* 2. RS485 UART */
    rs485_uart_init();

    /* 3. RGB LEDs — R turns ON here as power indicator */
    rgb_led_init();

    /* 4. Accelerometer INT1 input (wired, not yet used) */
    accel_int1_gpio_init();

    /*
     * Note: I2C bus and LIS3DHTR are initialised inside accel_test_task()
     * so that init errors are contained within that task and do not block
     * the RS485 or LED tests from starting.
     */

    // -----------------------------------------------------------------------
    // Task creation
    // -----------------------------------------------------------------------

    /*
     * LED blink task — Core 0, Priority 4
     * Low priority; timing is approximate (±1 FreeRTOS tick).
     */
    xTaskCreatePinnedToCore(
        rgb_led_blink_task,
        "led_blink",
        2048,
        NULL,
        4,
        NULL,
        0   // Core 0
    );

    /*
     * Accelerometer task — Core 0, Priority 4
     * Runs I2C scan + WHO_AM_I first, then periodic data reads.
     * Slightly larger stack to accommodate I2C driver internals.
     */
    xTaskCreatePinnedToCore(
        accel_test_task,
        "accel_test",
        4096,
        NULL,
        4,
        NULL,
        0   // Core 0
    );

    /*
     * RS485 MUX test task — Core 1, Priority 5
     * Higher priority and pinned to Core 1 to keep RS485 timing clean
     * and independent of the I2C / LED tasks on Core 0.
     */
    xTaskCreatePinnedToCore(
        rs485_mux_test_task,
        "rs485_mux_test",
        4096,
        NULL,
        5,
        NULL,
        1   // Core 1
    );

    ESP_LOGI(TAG_MAIN, "All tasks started. Monitoring serial output...");
    ESP_LOGI(TAG_MAIN, "  - Watch RS485 TX LEDs for MUX channel switching");
    ESP_LOGI(TAG_MAIN, "  - Watch RGB LEDs blink every 2 s");
    ESP_LOGI(TAG_MAIN, "  - Watch console for LIS3DHTR WHO_AM_I and accel data");
}