/**
 * @file bms_monitor_task.c
 * @brief JK-BMS Battery Monitoring Task Implementation for KSC TCU V1.1
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "bms_monitor_task.h"
#include "bms_rs485_mux.h"
#include "jkbms_serial_storage.h"
#include "jk_bms.h"
#include "lis3dhtr.h"
#include "rgb_led.h"
#include "board_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#define TAG "BMS_MONITOR"

/* =========================================================================
 * TASK CONFIGURATION
 * ========================================================================= */

#define BMS_MONITOR_TASK_STACK  4096
#define BMS_MONITOR_TASK_PRIO   5
#define BMS_MONITOR_TASK_CORE   0

#define BMS_LED_TASK_STACK      2048
#define BMS_LED_TASK_PRIO       4
#define BMS_LED_TASK_CORE       0

#define BMS_DEVICE_ADDRESS      0x01

/* =========================================================================
 * SERIAL READ CONFIGURATION
 * ========================================================================= */

#define SERIAL_READ_MAX_RETRIES  5
#define SERIAL_READ_DELAY_MS     50

/* =========================================================================
 * MOTION DETECTION
 * =========================================================================
 * Primary:   LIS3DHTR hardware interrupt (current draw as fallback).
 * Threshold: >5 A on any battery indicates trike motion.
 * ========================================================================= */

#define MOTION_CURRENT_THRESHOLD_A  5

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static QueueHandle_t  bms_data_queue          = NULL;
static TaskHandle_t   bms_monitor_task_handle = NULL;
static TaskHandle_t   bms_led_task_handle     = NULL;
static SemaphoreHandle_t led_status_mutex     = NULL;

static jk_device_t    jk_devices[BMS_BATTERY_COUNT];
static jk_data_t      jk_data_temp;
static bms_system_stats_t system_stats        = {0};
static bool           task_running            = false;

static volatile uint8_t  batteries_read_ok    = 0;
static volatile bool     led_status_updated   = false;
static volatile bool     trike_in_motion      = false;
static volatile uint8_t  s_batteries_ok_count = 0;

/** Cached serial numbers for fallback when a battery is unreadable */
static char cached_serials[BMS_BATTERY_COUNT][16] = {{0}};

/* =========================================================================
 * LED STATUS TASK
 * ========================================================================= */

/**
 * @brief LED status task — mirrors BMS read health via the RGB LED system.
 *
 * Delegates actual LED control to rgb_led_set_alert() rather than driving
 * GPIO directly; this keeps LED logic in one place.
 *
 * @param[in] pvParameters  Unused.
 */
static void bms_led_status_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED status task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));

        uint8_t ok_count = s_batteries_ok_count;

        /* Set alert mode based on battery health.
         * Note: SOC-based alerts are set by the MQTT publish task which has
         * the computed average SOC; BMS task only sets the battery-count alert. */
        if (ok_count == 0) {
            rgb_led_set_alert(ALERT_NO_BATTERIES);
        } else if (ok_count < BMS_BATTERY_COUNT) {
            /* Partial read — red fast blink handled by NO_BATTERIES pattern
             * using a different rate to distinguish */
            rgb_led_set_alert(ALERT_NO_BATTERIES);
        }
        /* Normal alert (ALERT_NORMAL) is set by the publish task */
    }
}

/* =========================================================================
 * SINGLE BATTERY READ
 * ========================================================================= */

/**
 * @brief Read all registers from one JK-BMS battery.
 *
 * Serial number is read first (up to SERIAL_READ_MAX_RETRIES attempts).
 * If the serial read fails all remaining registers are skipped and the
 * cached serial is used so the queue entry carries identification data.
 *
 * @param[in]  battery_id   Which battery to read.
 * @param[out] queued_data  Output structure to populate.
 * @return true on complete success, false if serial read failed.
 */
static bool read_single_battery(bms_battery_id_t battery_id,
                                 bms_queued_data_t *queued_data)
{
    memset(queued_data, 0, sizeof(bms_queued_data_t));
    queued_data->battery_id = battery_id;
    queued_data->timestamp  = (uint32_t)(esp_timer_get_time() / 1000);
    queued_data->status     = BMS_STATUS_NO_DATA;

    if (bms_rs485_switch_to_battery(battery_id) != ESP_OK) {
        ESP_LOGE(TAG, "MUX switch failed for battery %d", battery_id + 1);
        system_stats.battery_stats[battery_id].failed_reads++;
        bms_serial_storage_get(battery_id, queued_data->serial_no);
        return false;
    }

    memset(&jk_data_temp, 0, sizeof(jk_data_temp));

    //ESP_LOGI(TAG, "--- Reading Battery %d ---", battery_id + 1);

    /* ----------------------------------------------------------------
     * Serial number — up to SERIAL_READ_MAX_RETRIES attempts
     * ---------------------------------------------------------------- */
    int8_t ret = -1;
    bool serial_ok = false;

    for (uint8_t attempt = 0;
         attempt < SERIAL_READ_MAX_RETRIES && !serial_ok;
         attempt++) {
        vTaskDelay(pdMS_TO_TICKS(SERIAL_READ_DELAY_MS));
        ret = jk_bms_read_serial(&jk_devices[battery_id],
                                  jk_data_temp.serial_no);
        if (ret == MODBUS_RTU_OK) {
            serial_ok = true;
        }
    }

    if (!serial_ok) {
        //ESP_LOGE(TAG, "Serial read failed after %d attempts",
                 //SERIAL_READ_MAX_RETRIES);
        if (bms_serial_storage_get(battery_id,
                                    queued_data->serial_no) != ESP_OK) {
            snprintf(queued_data->serial_no, sizeof(queued_data->serial_no),
                     "BMS%d_UNKNOWN", battery_id + 1);
        }
        system_stats.battery_stats[battery_id].failed_reads++;
        return false;
    }

    /* ----------------------------------------------------------------
     * Helper macro: read a register with one retry on failure
     * ---------------------------------------------------------------- */
#define READ_REG(fn, dst, label)                                   \
    do {                                                           \
        uint8_t _att = 0;                                         \
        do {                                                       \
            vTaskDelay(pdMS_TO_TICKS(50));                        \
            ret = (fn);                                            \
        } while (ret != MODBUS_RTU_OK && _att++ < 1);             \
        if (ret != MODBUS_RTU_OK) {                               \
            ESP_LOGE(TAG, "Failed to read " label " (err %d)", ret); \
        }                                                          \
    } while (0)

    READ_REG(jk_bms_read_device_id(&jk_devices[battery_id],
                                    jk_data_temp.model_no),
             jk_data_temp.model_no, "Device ID");

    READ_REG(jk_bms_read_device_address(&jk_devices[battery_id],
                                         &jk_data_temp.device_address),
             jk_data_temp.device_address, "Device Address");

    READ_REG(jk_bms_read_cells_present(&jk_devices[battery_id],
                                        &jk_data_temp.cells_present),
             jk_data_temp.cells_present, "Cells Present");

    READ_REG(jk_bms_read_cell_count(&jk_devices[battery_id],
                                     &jk_data_temp.cell_count),
             jk_data_temp.cell_count, "Cell Count");

    READ_REG(jk_bms_read_cells_diff(&jk_devices[battery_id],
                                     &jk_data_temp.cells_diff),
             jk_data_temp.cells_diff, "Cells Diff");

    READ_REG(jk_bms_read_soc(&jk_devices[battery_id],
                               &jk_data_temp.soc),
             jk_data_temp.soc, "SOC");

    READ_REG(jk_bms_read_soh(&jk_devices[battery_id],
                               &jk_data_temp.soh),
             jk_data_temp.soh, "SOH");

    READ_REG(jk_bms_read_batt_voltage(&jk_devices[battery_id],
                                       &jk_data_temp.batt_v),
             jk_data_temp.batt_v, "Voltage");

    READ_REG(jk_bms_read_batt_current(&jk_devices[battery_id],
                                       &jk_data_temp.batt_i),
             jk_data_temp.batt_i, "Current");

    READ_REG(jk_bms_read_batt_power(&jk_devices[battery_id],
                                     &jk_data_temp.batt_power),
             jk_data_temp.batt_power, "Power");

    READ_REG(jk_bms_read_batt_remaining_capacity(
                 &jk_devices[battery_id],
                 &jk_data_temp.remaining_capacity),
             jk_data_temp.remaining_capacity, "Remaining Cap");

    READ_REG(jk_bms_read_charge_discharge_stat(
                 &jk_devices[battery_id],
                 &jk_data_temp.charge_stat,
                 &jk_data_temp.discharge_stat),
             jk_data_temp.charge_stat, "Charge/Discharge Stat");

    READ_REG(jk_bms_read_charge_cycles(&jk_devices[battery_id],
                                        &jk_data_temp.charge_cycles),
             jk_data_temp.charge_cycles, "Charge Cycles");

    READ_REG(jk_bms_read_alarms(&jk_devices[battery_id],
                                 &jk_data_temp.alarms),
             jk_data_temp.alarms, "Alarms");

#undef READ_REG

    /* ----------------------------------------------------------------
     * Copy data to queue structure
     * ---------------------------------------------------------------- */
    queued_data->device_address     = jk_data_temp.device_address;
    queued_data->batt_v             = jk_data_temp.batt_v;
    queued_data->batt_i             = jk_data_temp.batt_i;
    queued_data->batt_power         = jk_data_temp.batt_power;
    queued_data->remaining_capacity = jk_data_temp.remaining_capacity;
    queued_data->cell_count         = jk_data_temp.cell_count;
    queued_data->cells_present      = jk_data_temp.cells_present;
    queued_data->cells_diff         = jk_data_temp.cells_diff;
    queued_data->soc                = jk_data_temp.soc;
    queued_data->soh                = jk_data_temp.soh;
    queued_data->charge_stat        = jk_data_temp.charge_stat;
    queued_data->discharge_stat     = jk_data_temp.discharge_stat;
    queued_data->charge_cycles      = jk_data_temp.charge_cycles;
    queued_data->alarms             = jk_data_temp.alarms;
    memcpy(queued_data->model_no, jk_data_temp.model_no,
           sizeof(queued_data->model_no));
    memcpy(queued_data->serial_no, jk_data_temp.serial_no,
           sizeof(queued_data->serial_no));

    /* Serial change detection (battery swap) */
    char old_serial[16];
    bool force_nvs = false;
    if (bms_serial_storage_get(battery_id, old_serial) == ESP_OK) {
        if (strcmp(old_serial, jk_data_temp.serial_no) != 0) {
            force_nvs = true;
            ESP_LOGW(TAG, "Battery %d serial changed: %s → %s",
                     battery_id + 1, old_serial, jk_data_temp.serial_no);
        }
    }
    bms_serial_storage_save(battery_id, jk_data_temp.serial_no, force_nvs);
    strncpy(cached_serials[battery_id], jk_data_temp.serial_no, 15);
    cached_serials[battery_id][15] = '\0';

    queued_data->status = BMS_STATUS_OK;
    system_stats.battery_stats[battery_id].successful_reads++;
    system_stats.battery_stats[battery_id].last_success_timestamp =
        queued_data->timestamp;

    ESP_LOGI(TAG, "Battery %d: V=%" PRIu32 "mV I=%" PRId32 "mA SOC=%u%%",
             battery_id + 1, queued_data->batt_v,
             queued_data->batt_i, queued_data->soc);

    return true;
}

/* =========================================================================
 * MAIN BMS MONITORING TASK
 * ========================================================================= */

/**
 * @brief Cyclic battery read task running on Core 0.
 *
 * Each cycle reads all BMS_BATTERY_COUNT batteries sequentially and
 * enqueues the results for the MQTT publish task.  If the queue is full
 * the oldest entry is discarded (overwrite semantics).
 *
 * @param[in] pvParameters  Unused.
 */
static void bms_monitor_task(void *pvParameters)
{
    bms_queued_data_t battery_data;
    s_batteries_ok_count = 0;

    ESP_LOGI(TAG, "BMS monitor task started on Core %d", xPortGetCoreID());
    task_running = true;

    /* Current buffer for motion fallback */
    int32_t current_ma[BMS_BATTERY_COUNT] = {0};

    while (1) {
        uint32_t loop_start = xTaskGetTickCount();
        s_batteries_ok_count = 0;

        for (bms_battery_id_t batt_id = BMS_BATTERY_1;
             batt_id < BMS_BATTERY_COUNT;
             batt_id++) {

            bool ok = read_single_battery(batt_id, &battery_data);

            if (ok) {
                s_batteries_ok_count++;
            } else {
                system_stats.battery_stats[batt_id].last_failure_timestamp =
                    battery_data.timestamp;
            }

            /* Store current for motion fallback */
            current_ma[batt_id] = battery_data.batt_i;

            /* Enqueue — overwrite oldest on full */
            if (xQueueSend(bms_data_queue, &battery_data, 0) != pdPASS) {
                bms_queued_data_t dummy;
                if (xQueueReceive(bms_data_queue, &dummy, 0) == pdPASS) {
                    system_stats.queue_overruns++;
                    xQueueSend(bms_data_queue, &battery_data,
                               pdMS_TO_TICKS(10));
                } else {
                    system_stats.queue_full_events++;
                }
            } else {
                system_stats.total_readings++;
            }

            bms_rs485_disable_battery(batt_id);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* ----------------------------------------------------------------
         * Motion detection
         * Primary:  LIS3DHTR hardware interrupt
         * Fallback: battery current > threshold
         * ---------------------------------------------------------------- */
        bool accel_motion = lis3dhtr_is_in_motion();

        bool current_motion =
            (llabs(current_ma[0]) > MOTION_CURRENT_THRESHOLD_A * 1000) ||
            (llabs(current_ma[1]) > MOTION_CURRENT_THRESHOLD_A * 1000) ||
            (llabs(current_ma[2]) > MOTION_CURRENT_THRESHOLD_A * 1000);

        trike_in_motion = accel_motion || current_motion;

/*        ESP_LOGI(TAG, "Cycle done: %d/%d OK  motion=%s (accel=%d curr=%d)",
                 s_batteries_ok_count, BMS_BATTERY_COUNT,
                 trike_in_motion ? "YES" : "NO",
                 accel_motion, current_motion);*/

        /* Wait until BMS_READ_INTERVAL_MS has elapsed from loop start */
        uint32_t elapsed = (xTaskGetTickCount() - loop_start) *
                           portTICK_PERIOD_MS;
        if (elapsed < BMS_READ_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(BMS_READ_INTERVAL_MS - elapsed));
        }
    }
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

QueueHandle_t bms_monitor_get_queue_handle(void)
{
    return bms_data_queue;
}

jk_device_t *bms_monitor_get_devices(void)
{
    return jk_devices;
}

uint8_t bms_monitor_get_ok_count(void)
{
    return s_batteries_ok_count;
}

void bms_monitor_get_stats(bms_system_stats_t *stats)
{
    if (stats) {
        memcpy(stats, &system_stats, sizeof(bms_system_stats_t));
    }
}

void bms_monitor_reset_stats(void)
{
    memset(&system_stats, 0, sizeof(bms_system_stats_t));
}

bool bms_monitor_task_is_running(void)
{
    return task_running;
}

bool check_trike_motion(void)
{
    return trike_in_motion;
}

uint32_t bms_monitor_task_get_stack_hwm(void)
{
    return (bms_monitor_task_handle != NULL) ?
           uxTaskGetStackHighWaterMark(bms_monitor_task_handle) : 0;
}

bool bms_monitor_set_discharge_all(uint8_t state)
{
    bool all_ok = true;
    const char *label = (state == JK_MOSFET_ENABLE) ? "ENABLE" : "DISABLE";

    for (bms_battery_id_t id = BMS_BATTERY_1;
         id < BMS_BATTERY_COUNT; id++) {

        if (bms_rs485_switch_to_battery(id) != ESP_OK) {
            all_ok = false;
            bms_rs485_disable_battery(id);
            continue;
        }

        int ret = -1;
        for (int retry = 0; retry < 2; retry++) {
            if (retry > 0) vTaskDelay(pdMS_TO_TICKS(100));
            ret = jk_bms_toggle_discharge(&jk_devices[id], state);
            if (ret == MODBUS_RTU_OK) break;
        }

        if (ret != MODBUS_RTU_OK) {
            ESP_LOGE(TAG, "Discharge %s failed for battery %d",
                     label, id + 1);
            all_ok = false;
        }

        bms_rs485_disable_battery(id);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return all_ok;
}

esp_err_t bms_monitor_task_init(struct modbus_rtu_interface_s *interface)
{
    if (!interface || !interface->read || !interface->write) {
        ESP_LOGE(TAG, "Invalid Modbus interface");
        return ESP_ERR_INVALID_ARG;
    }

    bms_rs485_mux_init();

    esp_err_t ret = bms_serial_storage_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Serial storage init failed — continuing");
    }
    bms_serial_storage_load(cached_serials);

    bms_data_queue = xQueueCreate(BMS_TOTAL_QUEUE_SIZE,
                                   sizeof(bms_queued_data_t));
    if (!bms_data_queue) {
        ESP_LOGE(TAG, "Failed to create BMS data queue");
        return ESP_FAIL;
    }

    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
        jk_devices[i].modbus.device_addr = BMS_DEVICE_ADDRESS;
        jk_devices[i].modbus.interface   = *interface;
    }

    /* Create LED status mutex and task */
    led_status_mutex = xSemaphoreCreateMutex();
    if (!led_status_mutex) {
        vQueueDelete(bms_data_queue);
        return ESP_FAIL;
    }

/*    BaseType_t led_created = xTaskCreatePinnedToCore(
        bms_led_status_task, "bms_led",
        BMS_LED_TASK_STACK, NULL,
        BMS_LED_TASK_PRIO, &bms_led_task_handle,
        BMS_LED_TASK_CORE
    );
    if (led_created != pdPASS) {
        vQueueDelete(bms_data_queue);
        vSemaphoreDelete(led_status_mutex);
        return ESP_FAIL;
    }*/

    /* Create BMS monitor task */
    BaseType_t mon_created = xTaskCreatePinnedToCore(
        bms_monitor_task, "bms_monitor",
        BMS_MONITOR_TASK_STACK, NULL,
        BMS_MONITOR_TASK_PRIO, &bms_monitor_task_handle,
        BMS_MONITOR_TASK_CORE
    );
    if (mon_created != pdPASS) {
        vQueueDelete(bms_data_queue);
        vSemaphoreDelete(led_status_mutex);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMS monitor initialised — queue depth %d, Core %d",
             BMS_TOTAL_QUEUE_SIZE, BMS_MONITOR_TASK_CORE);
    return ESP_OK;
}