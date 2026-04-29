/**
 * @file main.c
 * @brief KSC TCU V1.1 — ESP32-S3 Multi-Battery Monitor System
 *
 * Application entry point and MQTT publish task for the Trike Control Unit
 * Version 1.1.  Combines JK-BMS battery monitoring, Quectel EG915N MQTT
 * telemetry, GPS position streaming, trike power control, accelerometer
 * motion detection, and analogue sensor readings.
 *
 * Hardware: TCU V1.1 board (ESP32-S3, MC74HC4052ADG RS-485 MUX,
 *           INA240A1PWR current sense, LIS3DHTR accelerometer).
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <math.h>

#include "board_config.h"
#include "version.h"
#include "rgb_led.h"
#include "trike_sensors.h"
#include "lis3dhtr.h"
#include "power_trike_ctrl.h"
#include "Quectel_mqtt.h"
#include "Quectel_gps.h"
#include "bms_monitor_types.h"
#include "bms_monitor_task.h"
#include "factory_test.h"
#include "tcu_nvs_creds.h"

#define TAG "MAIN"

/* =========================================================================
 * TIMING CONFIGURATION
 * ========================================================================= */

#define PUBLISH_INTERVAL_MS          (60  * 1000)
#define GPS_UPDATE_INTERVAL_MS       (30  * 1000)
#define BMS_HEALTH_CHECK_INTERVAL_MS (60  * 1000)
#define KEEPALIVE_INTERVAL_MS        (30  * 1000)
#define MODEM_CHECK_INTERVAL_MS      (60  * 1000)
#define URC_POLL_INTERVAL_MS          100

/* =========================================================================
 * SAFETY THRESHOLDS
 * ========================================================================= */

/** Cross-battery voltage deviation that triggers trike shutdown (mV) */
#define BATT_VOLTAGE_DEVIATION_MV    5000

/** Average SOC below which WARN alert is activated (%) */
#define SOC_WARN_THRESHOLD_PCT       20

/** Average SOC below which CRIT alert is activated (%) */
#define SOC_CRIT_THRESHOLD_PCT       10

/* =========================================================================
 * WATCHDOG CONFIGURATION
 * =========================================================================
 * 300 s (5 min) is appropriate for production.
 * The MQTT publish task resets the watchdog every ~1 s in the main loop.
 * If the publish task hangs for more than 5 minutes the system restarts.
 * ========================================================================= */

#define WATCHDOG_TIMEOUT_S           300

/* =========================================================================
 * MODULE-LEVEL GLOBALS
 * ========================================================================= */
char deviceSerial[13] = {0};
int     slave_id  = 0;

static uint8_t returned_average_soc       = 0;
static volatile bool publishing_status_only = false;
static volatile bool got_gps_fix= false;

extern bool check_trike_motion(void);
extern bool get_gsm_signal_quality(int *rssi_out, int *dbm_out);

/* =========================================================================
 * CHIP ID (full 6-byte MAC hex string)
 * ========================================================================= */

/**
 * @brief Build a 12-character unique device identifier from the ESP32-S3
 *        base MAC address.
 *
 * Uses all 6 MAC bytes in hex format (e.g. "A4CF12B3E501") — guaranteed
 * unique and matches the label printed on the ESP32-S3 module.
 *
 * @param[out] buf   Buffer to receive the string (minimum 13 bytes).
 * @param[in]  size  Buffer size.
 */
void getChipIdString(char *buf, size_t size)
{
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);
    snprintf(buf, size, "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device serial: %s", buf);
}

/* =========================================================================
 * BATTERY JSON PAYLOAD
 * ========================================================================= */

/**
 * @brief Build the telemetry JSON payload for one battery.
 *
 * If the battery's voltage deviates more than BATT_VOLTAGE_DEVIATION_MV
 * from any other battery, field Bx_P26 is set to 1 (alarm).  When the
 * deviation clears, Bx_P26 is set to 0 so ThingsBoard can auto-clear
 * the alarm.
 *
 * @param[in] battery_data      Data for the battery being published.
 * @param[in] all_battery_data  Full array of all battery readings (for
 *                              cross-battery voltage deviation check).
 * @param[in] batteries_read    Number of valid entries in all_battery_data.
 * @return Heap-allocated JSON string.  Caller must free() after use.
 *         Returns NULL if battery_data is NULL.
 */
char *create_battery_json_payload(bms_queued_data_t *battery_data,
                                   bms_queued_data_t *all_battery_data,
                                   int batteries_read)
{
    if (!battery_data) return NULL;

    cJSON *root = cJSON_CreateObject();
    char  prefix[16], key[32];

    snprintf(prefix, sizeof(prefix), "B%d", battery_data->battery_id + 1);

#define ADD_NUM(field, value) \
    snprintf(key, sizeof(key), "%s_" #field, prefix); \
    cJSON_AddNumberToObject(root, key, (value))

    snprintf(key, sizeof(key), "%s_P12", prefix);
    cJSON_AddNumberToObject(root, key,
        roundf((float)battery_data->batt_v / 10.0f) / 100.0f);

    snprintf(key, sizeof(key), "%s_P13", prefix);
    cJSON_AddNumberToObject(root, key,
        roundf((float)battery_data->batt_i / 10.0f) / 100.0f);

    snprintf(key, sizeof(key), "%s_P14", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->soc);

    snprintf(key, sizeof(key), "%s_P15", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->soh);

    snprintf(key, sizeof(key), "%s_P16", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->charge_cycles);

    snprintf(key, sizeof(key), "%s_P17", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->status);

    snprintf(key, sizeof(key), "%s_P18", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->charge_stat);

    snprintf(key, sizeof(key), "%s_P19", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->discharge_stat);

    snprintf(key, sizeof(key), "%s_P20", prefix);
    cJSON_AddStringToObject(root, key, battery_data->serial_no);

    snprintf(key, sizeof(key), "%s_P21", prefix);
    cJSON_AddNumberToObject(root, key,
        roundf((float)battery_data->batt_power / 10.0f) / 100.0f);

    snprintf(key, sizeof(key), "%s_P22", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->cells_diff);

    snprintf(key, sizeof(key), "%s_P23", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->alarms);

    snprintf(key, sizeof(key), "%s_P24", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->cell_count);

    snprintf(key, sizeof(key), "%s_P25", prefix);
    cJSON_AddNumberToObject(root, key,
        roundf((float)battery_data->remaining_capacity / 10.0f) / 100.0f);

    /* Cross-battery voltage deviation check */
    bool voltage_anomaly = false;
    if (all_battery_data && batteries_read > 1 &&
        battery_data->status == BMS_STATUS_OK) {
        for (int j = 0; j < batteries_read; j++) {
            if (all_battery_data[j].battery_id == battery_data->battery_id)
                continue;
            if (all_battery_data[j].status != BMS_STATUS_OK)
                continue;
            int32_t diff = (int32_t)battery_data->batt_v -
                           (int32_t)all_battery_data[j].batt_v;
            if (diff < 0) diff = -diff;
            if ((uint32_t)diff > BATT_VOLTAGE_DEVIATION_MV) {
                voltage_anomaly = true;
                ESP_LOGW(TAG, "B%d voltage anomaly vs B%d: %" PRId32 " mV",
                         battery_data->battery_id + 1,
                         all_battery_data[j].battery_id + 1, diff);
                break;
            }
        }
    }

    /* Always publish P26 so ThingsBoard can clear the alarm when normal */
    snprintf(key, sizeof(key), "%s_P26", prefix);
    cJSON_AddNumberToObject(root, key, voltage_anomaly ? 1 : 0);

#undef ADD_NUM

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

/* =========================================================================
 * METADATA PAYLOAD
 * ========================================================================= */

/**
 * @brief Build the device metadata JSON payload for ThingsBoard.
 *
 * Fields published:
 *   "1"   — Mains present (1/0)  [always 1 on V1.1 — no mains detect pin]
 *   "2"   — Device serial (MAC string)
 *   "3"   — Valid battery count
 *   "4–7" — GPS latitude, longitude, altitude, satellites (if valid fix)
 *   "8"   — GPS fix quality
 *   "9"   — trike_power_resp_cmd (only during power-off pipeline)
 *   "10"  — power_confirmation (trike ON/OFF state)
 *   "11"  — Reset reason (omitted on clean power-on reset)
 *   "27"  — Average battery SOC (only when all batteries readable)
 *   "P28" — Trike voltage (V)
 *   "P29" — Trike current (A)
 *   "P30" — Powertrain temperature (°C)
 *   "P31" — Accelerometer roll (degrees)
 *   "P32" — Accelerometer pitch (degrees)
 *   "P33" — Motion state (0/1)
 *   "fw"  — Firmware version string
 *   "hw"  — Hardware version string
 *
 * @return Heap-allocated JSON string.  Caller must free() after use.
 */
char *create_metadata_payload()
{

    cJSON *root = cJSON_CreateObject();

    /* Field "1": mains present — TCU V1.1 has no mains sense pin;
     * always report 1 (board is powered when it is running) */
    cJSON_AddNumberToObject(root, "1", 1);

    /* Field "2": device serial */
    cJSON_AddStringToObject(root, "2", deviceSerial);

    /* Field "3": valid battery count */
    cJSON_AddNumberToObject(root, "3", bms_monitor_get_ok_count());
    
	/* Fields P37–P38: GSM signal quality and GPS satellite count */
    int gsm_rssi = 99, gsm_dbm = -999;
    if (get_gsm_signal_quality(&gsm_rssi, &gsm_dbm)) {
        cJSON_AddNumberToObject(root, "P37", gsm_dbm);   /* GSM signal dBm */
    } else {
        cJSON_AddNumberToObject(root, "P37", -999);      /* No signal      */
    }    

    /* Fields "4–8": GPS position */
    gps_position_t gps_pos;
    memset(&gps_pos, 0, sizeof(gps_pos));

    if (gps_get_latest_position(&gps_pos) == ESP_OK && gps_pos.valid) {
        cJSON_AddNumberToObject(root, "4", gps_pos.latitude);
        cJSON_AddNumberToObject(root, "5", gps_pos.longitude);
        cJSON_AddNumberToObject(root, "6", gps_pos.altitude);
        cJSON_AddNumberToObject(root, "7", gps_pos.satellites);
    }
    cJSON_AddNumberToObject(root, "8", gps_pos.fix);

    // -----------------------------------------------------------------------
    //Field "9" only during trike power-off pipeline
    //Value responds to confirm if the trike will switch off (responds FALSE if power off command executed, reponds TRUE if command is rejected because trike is in motion)
    //Field "9" = TRUE means: "received the off command but trike is still moving"
    //Field "9" = FALSE means: "the power-off command was carried out"
    // -----------------------------------------------------------------------
    bool resp_cmd_val = false;
    if (trike_ctrl_consume_resp_cmd_flag(&resp_cmd_val)) {
        cJSON_AddBoolToObject(root, "9", resp_cmd_val);
    }

    // -----------------------------------------------------------------------
    // REQ 1: Field "10" - power_confirmation from trike_power_ctrl (NVS-backed); Also the current actual
    //power status of trike (TRUE when trike authorized, FALSE when trike unauthorized)
    // -----------------------------------------------------------------------
    cJSON_AddBoolToObject(root, "10", trike_ctrl_get_power_confirmation());

    /* Field "11": reset reason (omit on clean power-on) */
    if (!power_mgmt_is_power_on_reset()) {
        char reason[32];
        power_mgmt_get_reset_reason(reason, sizeof(reason));
        cJSON_AddStringToObject(root, "11", reason);
    }

    /* Field "27": average SOC — only when all batteries are valid */
    if (!publishing_status_only) {
        returned_average_soc = get_average_battery_soc();
        if (bms_monitor_get_ok_count() == BMS_BATTERY_COUNT &&
            returned_average_soc > 0) {
            cJSON_AddNumberToObject(root, "27", returned_average_soc);
        } else {
            cJSON_AddNumberToObject(root, "27", 0);
        }
    }

    /* Fields P28–P32: analogue sensor readings */
    trike_sensor_data_t sensors;
    if (trike_sensors_read_all(&sensors) == ESP_OK && sensors.valid) {
        cJSON_AddNumberToObject(root, "P28", sensors.voltage_v);
        cJSON_AddNumberToObject(root, "P29", sensors.current_a);
        cJSON_AddNumberToObject(root, "P30", sensors.temperature_c);
    }

	/* Fields P31–P35: accelerometer — vibration and axis data (roll/pitch NOT published) */
	lis3dhtr_motion_t motion;
	if (lis3dhtr_read_motion(&motion) == ESP_OK) {
	    cJSON_AddNumberToObject(root, "P31", motion.x_mg);         /* X accel mg  */
	    cJSON_AddNumberToObject(root, "P32", motion.y_mg);         /* Y accel mg  */
	    cJSON_AddNumberToObject(root, "P33", motion.z_mg);         /* Z accel mg  */
	    cJSON_AddNumberToObject(root, "P34", motion.accel_rms_mg); /* RMS mg      */
	    cJSON_AddNumberToObject(root, "P35", motion.accel_mag_mg); /* Magnitude mg      */
	    cJSON_AddNumberToObject(root, "P36", motion.in_motion ? 1 : 0);
	}    

    /* Firmware and hardware version */
    cJSON_AddStringToObject(root, "fw", FW_VERSION_STR);
    cJSON_AddStringToObject(root, "hw", HW_VERSION_STR);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

/* =========================================================================
 * RS-485 UART INITIALISATION
 * ========================================================================= */

/**
 * @brief Initialise the RS-485 UART and driver.
 *
 * RS-485 half-duplex mode is used.  The MC74HC4052ADG MUX handles bus
 * routing; the UART driver handles TX/RX direction automatically.
 */
static void rs485_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = BOARD_RS485_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(BOARD_RS485_UART_NUM,
                                        256 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_RS485_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_RS485_UART_NUM,
                                  BOARD_RS485_TX_PIN,
                                  BOARD_RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(BOARD_RS485_UART_NUM,
                                   UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "RS-485 UART%d initialised (TX=IO%d RX=IO%d)",
             BOARD_RS485_UART_NUM,
             BOARD_RS485_TX_PIN,
             BOARD_RS485_RX_PIN);
}

static int rs485_write(const uint8_t *data, const uint16_t len)
{
    int n = uart_write_bytes(BOARD_RS485_UART_NUM, data, len);
    uart_wait_tx_done(BOARD_RS485_UART_NUM, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
    return n;
}

static int rs485_read(uint8_t *dst, uint16_t len)
{
    return uart_read_bytes(BOARD_RS485_UART_NUM, dst, len,
                           pdMS_TO_TICKS(50));
}

/* =========================================================================
 * VOLTAGE ANOMALY CHECK
 * ========================================================================= */

/**
 * @brief Check for cross-battery voltage deviation and initiate trike
 *        shutdown if an anomaly is detected.
 *
 * Called after successfully publishing battery data.
 *
 * @param[in] all_battery_data  Array of battery data from the last read cycle.
 * @param[in] batteries_read    Number of valid entries.
 */
void check_voltage_anomaly_and_act(bms_queued_data_t *all_battery_data,
                                    int batteries_read)
{
    bool anomaly = false;

    for (int i = 0; i < batteries_read && !anomaly; i++) {
        if (all_battery_data[i].status != BMS_STATUS_OK) continue;
        for (int j = i + 1; j < batteries_read; j++) {
            if (all_battery_data[j].status != BMS_STATUS_OK) continue;
            int32_t diff = (int32_t)all_battery_data[i].batt_v -
                           (int32_t)all_battery_data[j].batt_v;
            if (diff < 0) diff = -diff;
            if ((uint32_t)diff > BATT_VOLTAGE_DEVIATION_MV) {
                ESP_LOGE(TAG, "Voltage anomaly B%d vs B%d = %" PRId32 " mV",
                         all_battery_data[i].battery_id + 1,
                         all_battery_data[j].battery_id + 1, diff);
                anomaly = true;
                break;
            }
        }
    }

    if (!anomaly) return;

/*    if( rgb_led_set_alert(ALERT_VOLTAGE_ANOMALY))  ESP_LOGI(TAG, "Switched RGB LED status");
    else{
		ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
	}*/
    ESP_LOGW(TAG, "Triggering trike poweroff due to voltage anomaly");
    trike_ctrl_handle_command(1);
}

/* =========================================================================
 * URC PROCESSING TASK
 * ========================================================================= */

/**
 * @brief Dedicated task that processes queued MQTT URC notifications.
 *
 * Runs on Core 1.  Receives URC notification structs from the queue
 * populated by check_mqtt_urc(), then calls read_buffered_messages()
 * to parse the MQTT command and dispatch to the appropriate handler.
 *
 * @param[in] pvParameters  Unused.
 */
static void process_urcs_task(void *pvParameters)
{
    ESP_LOGI(TAG, "URC task started on Core %d", xPortGetCoreID());
    urc_notification_t notif;

    while (1) {
        if (xQueueReceive(urc_notification_queue, &notif,
                          pdMS_TO_TICKS(URC_POLL_INTERVAL_MS)) == pdPASS) {
            uint32_t age = (uint32_t)(xTaskGetTickCount() *
                           portTICK_PERIOD_MS) - notif.timestamp_ms;
            if (age > 1000) {
                ESP_LOGW(TAG, "URC latency %" PRIu32 " ms", age);
            }
            if (!read_buffered_messages(notif.recv_id)) {
                ESP_LOGE(TAG, "Failed to read slot %d", notif.recv_id);
            }
        } else {
            check_mqtt_urc();
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* =========================================================================
 * MQTT PUBLISH TASK
 * ========================================================================= */

/**
 * @brief Main MQTT publish task — runs on Core 1.
 *
 * Responsibilities:
 *   - Initialise modem MQTT connection and GPS streaming
 *   - Periodic battery telemetry and metadata publish (every 60 s)
 *   - Triage SOC-based LED alerts
 *   - Trike shutdown on low battery count or voltage anomaly
 *   - Modem health monitoring and recovery
 *   - Power event handling
 *
 * @param[in] pvParameters  Unused.
 */
static void mqtt_publish_task(void *pvParameters)
{
    QueueHandle_t bms_queue   = bms_monitor_get_queue_handle();
    QueueHandle_t power_queue = power_mgmt_get_event_queue();

    ESP_LOGI(TAG, "MQTT publish task started on Core %d", xPortGetCoreID());

    static bool low_battery_shutdown_triggered = false;

     //Initialise MQTT connection 
    if (!mqtt_subclient()) {
        ESP_LOGE(TAG, "mqtt_subclient failed — restarting");
        powerdown_modem();
        esp_restart();
    }

    //Initialise GPS NMEA streaming 
    if (!gps_mqtt_init()) {
        ESP_LOGW(TAG, "GPS init failed — continuing without GPS");
    }

    /* Timing state */
    uint32_t last_publish_ms      = 0;
    uint32_t last_bms_health_ms   = 0;
    uint32_t last_keepalive_ms    = 0;
    uint32_t last_modem_check_ms  = 0;
    uint32_t last_stack_check_ms  = 0;
    uint32_t last_queue_size      = 0;

    // Restore trike state on ThingsBoard 
    vTaskDelay(pdMS_TO_TICKS(2000));
    trike_ctrl_publish_state();

/*    // Register with watchdog 
    esp_task_wdt_add(NULL);*/

    // Initialise URC queue and start URC processing task 
    urc_queue_init();
    xTaskCreatePinnedToCore(process_urcs_task, "urc_proc",
                             4096, NULL, 5, NULL, 1);

/*     Transition to normal operation LED state 
    if(rgb_led_set_alert(ALERT_NORMAL)) ESP_LOGI(TAG, "Switched RGB LED status");
    else ESP_LOGE(TAG, "Failed to switch RGB LED status!");*/ 

    while (1) {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        //esp_task_wdt_reset();

        // Stack monitoring (every 60 s)

        if ((now - last_stack_check_ms) >= 60000) {
            uint32_t hwm = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "MQTT task stack HWM: %" PRIu32 " words", hwm);
            if (hwm < 256) {
                ESP_LOGE(TAG, "Stack critically low!");
            }
            last_stack_check_ms = now;
        }

/*        // Power events

        power_event_t pev;
        if (xQueueReceive(power_queue, &pev, 0) == pdPASS) {
            switch (pev.type) {
                case POWER_EVENT_MODEM_BROWNOUT:
                    ESP_LOGW(TAG, "Modem brownout — attempting recovery");
                    if (power_mgmt_recover_modem() != ESP_OK) {
                        ESP_LOGE(TAG, "Modem recovery failed — restarting");
                        esp_restart();
                    }
                    break;

                default:
                    break;
            }
        }*/

        // Keep-alive (every 30 s)

        if ((now - last_keepalive_ms) >= KEEPALIVE_INTERVAL_MS) {
            check_mqtt_link_status();
            last_keepalive_ms = now;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // BMS task health check (every 60 s)
 
        if ((now - last_bms_health_ms) >= BMS_HEALTH_CHECK_INTERVAL_MS) {
            UBaseType_t qs = uxQueueMessagesWaiting(bms_queue);
            if (qs == last_queue_size && qs == 0 &&
                bms_monitor_task_is_running()) {
                ESP_LOGW(TAG, "BMS task appears unhealthy");
            }
            last_queue_size   = qs;
            last_bms_health_ms = now;
        }

        // Modem health check (every 60 s) 
        
        if ((now - last_modem_check_ms) >= MODEM_CHECK_INTERVAL_MS) {
            if (!power_mgmt_check_modem_alive()) {
                ESP_LOGW(TAG, "Modem health check failed — queuing recovery");
                power_event_t ev = {
                    .type         = POWER_EVENT_MODEM_BROWNOUT,
                    .timestamp_ms = now
                };
                xQueueSend(power_queue, &ev, 0);
            }
            last_modem_check_ms = now;
        }

        // Periodic publish (every 60 s)

        if ((now - last_publish_ms) >= PUBLISH_INTERVAL_MS) {
            uint8_t ok_count = bms_monitor_get_ok_count();
            ESP_LOGI(TAG, "Publish tick — batteries_ok=%d", ok_count);

            if (ok_count == 0) {
                // No valid batteries — publish metadata only 
                publishing_status_only = true;
                //if(rgb_led_set_alert(ALERT_NO_BATTERIES)){ESP_LOGI(TAG, "Switched RGB LED status");}
    			//else { ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
				//}
                if (!mqtt_pubclient_status()) {
                    ESP_LOGE(TAG, "Metadata-only publish failed");
                }
                else{
					if(!got_gps_fix){
		                if(rgb_led_set_alert(ALERT_LOW_SOC_WARN)){ESP_LOGI(TAG, "Switched RGB LED status");}
		    			else { ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
						}						
					}					
                }

            } else {
                publishing_status_only = false;

                if (!mqtt_pubclient_battery()) {
                    ESP_LOGE(TAG, "Battery publish failed");
                } else {
                    ESP_LOGI(TAG, "Battery data published");

                    // Trike shutdown if fewer than all batteries readable 
                    if (ok_count < BMS_BATTERY_COUNT &&
                        !low_battery_shutdown_triggered) {
                        ESP_LOGE(TAG,
                                 "Only %d/%d batteries readable — shutdown",
                                 ok_count, BMS_BATTERY_COUNT);
                        low_battery_shutdown_triggered = true;
                        trike_ctrl_handle_command(1);
                    } else if (ok_count == BMS_BATTERY_COUNT) {
                        low_battery_shutdown_triggered = false;
                    }
                }

                // SOC-based LED alert 
                uint8_t avg_soc = get_average_battery_soc();
                bool success = true;
                if (ok_count == BMS_BATTERY_COUNT) {
                    if (avg_soc > 0 && avg_soc <= SOC_CRIT_THRESHOLD_PCT) {
                        //success = rgb_led_set_alert(ALERT_LOW_SOC_CRIT);
                    } else if (avg_soc > SOC_CRIT_THRESHOLD_PCT &&
                               avg_soc <= SOC_WARN_THRESHOLD_PCT) {
                        //success = rgb_led_set_alert(ALERT_LOW_SOC_WARN);
                    } else if (trike_ctrl_get_power_confirmation()) {
                        //success = rgb_led_set_alert(ALERT_NORMAL);
                    } else {
                       //success = rgb_led_set_alert(ALERT_TRIKE_OFF);
                    }
                }
                //if(!success)ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
            }

            last_publish_ms = now;
            vTaskDelay(pdMS_TO_TICKS(10));
            check_mqtt_link_status();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* =========================================================================
 * STATISTICS TASK
 * ========================================================================= */

/**
 * @brief Periodic statistics logging task (every 30 s).
 *
 * @param[in] pvParameters  Unused.
 */
static void stats_monitor_task(void *pvParameters)
{
    bms_system_stats_t stats;
    ESP_LOGI(TAG, "Stats task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        bms_monitor_get_stats(&stats);
        ESP_LOGI(TAG, "=== BMS Statistics ===");
        ESP_LOGI(TAG, "Total: %" PRIu32 "  Overruns: %" PRIu32,
                 stats.total_readings, stats.queue_overruns);

        for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
            ESP_LOGI(TAG, "  B%d OK=%" PRIu32 " FAIL=%" PRIu32
                     " TO=%" PRIu32 " CRC=%" PRIu32,
                     i + 1,
                     stats.battery_stats[i].successful_reads,
                     stats.battery_stats[i].failed_reads,
                     stats.battery_stats[i].timeout_errors,
                     stats.battery_stats[i].crc_errors);
        }

        if (urc_notification_queue) {
            UBaseType_t uq = uxQueueMessagesWaiting(urc_notification_queue);
            if (uq > 5) {
                ESP_LOGW(TAG, "URC queue depth %d!", (int)uq);
            }
        }
        ESP_LOGI(TAG, "======================");
    }
}

/* =========================================================================
 * 1 Hz DIAGNOSTIC LOGGING TASK (lab testing only)
 * =========================================================================
 * Logs all sensor readings at 1 Hz to JTAG serial for bench verification.
 * Remove or gate behind a compile flag before production deployment.
 * ========================================================================= */

static void diag_log_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Diag log task started on Core %d", xPortGetCoreID());

    while (1) {
		ESP_LOGI(TAG, "======================");
		ESP_LOGI(TAG, "PRINTING LOGS AT 1HZ");
		ESP_LOGI(TAG, "======================");
		/* --- ESP32 Serial Number --- */
		ESP_LOGI(TAG,  "ESP32 Serial Number: %s", deviceSerial);
		
        /* --- Accelerometer --- */
        lis3dhtr_motion_t motion;
        if (lis3dhtr_read_motion(&motion) == ESP_OK) {
            ESP_LOGI(TAG, "[DIAG][ACCEL] X=%.1fmg Y=%.1fmg Z=%.1fmg "
                     "RMS=%.1fmg Mag=%.1fmg Motion=%d Impact=%d FF=%d Mode=%s",
                     motion.x_mg, motion.y_mg, motion.z_mg,
                     motion.accel_rms_mg, motion.accel_mag_mg,
                     motion.in_motion, motion.impact_detected, motion.free_fall,
                     lis3dhtr_get_mode() == LIS3DHTR_MODE_TEST ? "TEST" : "PROD");
        } else {
            ESP_LOGW(TAG, "[DIAG][ACCEL] read failed");
        }

        /* --- ADC sensors --- */
        trike_sensor_data_t sensors;
        if (trike_sensors_read_all(&sensors) == ESP_OK && sensors.valid) {
            ESP_LOGI(TAG, "[DIAG][ADC]   V=%.2fV I=%.3fA T=%.1f°C",
                     sensors.voltage_v, sensors.current_a,
                     sensors.temperature_c);
        } else {
            ESP_LOGW(TAG, "[DIAG][ADC] one or more reads failed");
        }

        /* --- BMS batteries --- */
        uint8_t ok_count = bms_monitor_get_ok_count();
        ESP_LOGI(TAG, "[DIAG][BMS]   batteries_ok=%d/%d  avg_soc=%d%%",
                 ok_count, BMS_BATTERY_COUNT, get_average_battery_soc());

        /* --- GPS --- */
        gps_position_t gps;
        memset(&gps, 0, sizeof(gps));
        esp_err_t gps_ret = gps_get_latest_position(&gps);
        if (gps_ret == ESP_OK) {
			got_gps_fix= true;
            ESP_LOGI(TAG, "[DIAG][GPS]   lat=%.5f lon=%.5f alt=%.1fm "
                     "fix=%d sats=%d",
                     gps.latitude, gps.longitude, gps.altitude,
                     gps.fix, gps.satellites);
		    if(rgb_led_set_alert(ALERT_LOW_SOC_CRIT)){ESP_LOGI(TAG, "Switched RGB LED status");}
			else { ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
			}
        } else {
            ESP_LOGI(TAG, "[DIAG][GPS]   no fix (fix=%d)", gps.fix);
        }

        /* --- GSM signal quality --- */
        int gsm_rssi = 99, gsm_dbm = -999;
        if (get_gsm_signal_quality(&gsm_rssi, &gsm_dbm)) {
            ESP_LOGI(TAG, "[DIAG][GSM]   rssi=%d  signal=%ddBm",
                     gsm_rssi, gsm_dbm);
        } else {
            ESP_LOGI(TAG, "[DIAG][GSM]   no signal (rssi=99)");
        }

        /* --- Trike power state --- */
        ESP_LOGI(TAG, "[DIAG][PWR]   trike_on=%d  motion=%d  avg_soc=%d%%",
                 trike_ctrl_get_power_confirmation(),
                 check_trike_motion(),
                 get_average_battery_soc());
                 

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* =========================================================================
 * APP_MAIN
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  KSC TCU V1.1  —  ESP32-S3");
    ESP_LOGI(TAG, "  FW: %s   HW: %s", FW_VERSION_STR, HW_VERSION_STR);
    ESP_LOGI(TAG, "  Built: %s %s", FW_BUILD_DATE, FW_BUILD_TIME);
    ESP_LOGI(TAG, "========================================");

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

//    /* Watchdog — 300 s production timeout */
//    esp_task_wdt_config_t wdt = {
//        .timeout_ms    = WATCHDOG_TIMEOUT_S * 1000,
//        .idle_core_mask = 0,
//        .trigger_panic  = true,
//    };
//    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt));
//    ESP_LOGI(TAG, "Watchdog: %d s timeout", WATCHDOG_TIMEOUT_S);

    /* RGB LEDs — must be first so fault LEDs work during init */
    ESP_ERROR_CHECK(rgb_led_init());

    /* Chip serial */
    getChipIdString(deviceSerial, sizeof(deviceSerial));
    snprintf(HUB_NAME, sizeof(HUB_NAME), "TCU_%s", deviceSerial);

    /* LIS3DHTR accelerometer */
    ret = lis3dhtr_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Accelerometer init failed — motion detection degraded");
    }
    
/*    vTaskDelay(pdMS_TO_TICKS(200));
    // Explicitly switch to TEST mode so HPF is off and gravity is visible
    lis3dhtr_configure_mode(LIS3DHTR_MODE_TEST);*/

    /* ADC sensors */
    ESP_ERROR_CHECK(trike_sensors_init());

    /* Log reset reason */
    char reset_reason[32];
    power_mgmt_get_reset_reason(reset_reason, sizeof(reset_reason));
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason);

    /* RS-485 UART */
    rs485_uart_init();
    uart_modem_mutex_init();

    /* Modbus RTU interface */
    struct modbus_rtu_interface_s rtu_interface = {
        .write = rs485_write,
        .read  = rs485_read,
    };

    /* Assign interface to JK-BMS devices */
    jk_device_t *jk_devices = bms_monitor_get_devices();
    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
        jk_devices[i].modbus.interface = rtu_interface;
    }

    /* BMS monitoring task */
    ret = bms_monitor_task_init(&rtu_interface);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMS monitor init failed");
        esp_restart();
    }

    /* Factory / startup self-test */
    ESP_LOGI(TAG, "Running factory self-test...");
    esp_err_t test_result = factory_test_run();
    if (test_result != ESP_OK) {
        ESP_LOGW(TAG, "One or more factory tests failed — continuing anyway");
    }

    // Power and trike control 
    ret = power_trike_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power/trike init failed");
        esp_restart();
    }   

    //Application tasks 
    xTaskCreatePinnedToCore(mqtt_publish_task,  "mqtt_publish",
                             6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(stats_monitor_task, "stats_monitor",
                             3072, NULL, 3, NULL, 1);
                             
     //Diagnostic 1Hz logging task — remove before production 
    xTaskCreatePinnedToCore(diag_log_task, "diag_log",
                             4072, NULL, 2, NULL, 0);                         

//    ESP_LOGI(TAG, "All tasks started");
    ESP_LOGI(TAG, "========================================");

    /* Main loop — watchdog feed for app_main task */
    while (1) {
	    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}