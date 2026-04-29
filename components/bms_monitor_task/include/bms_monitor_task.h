/**
 * @file bms_monitor_task.h
 * @brief JK-BMS Battery Monitoring Task Interface for KSC TCU V1.1
 *
 * Manages the periodic RS-485 polling of up to BMS_BATTERY_COUNT JK-BMS
 * batteries via the MC74HC4052ADG analog multiplexer.  Read results are
 * placed onto a FreeRTOS queue for consumption by the MQTT publish task.
 *
 * Motion detection uses the LIS3DHTR accelerometer as primary source with
 * battery current draw as a secondary fallback.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef BMS_MONITOR_TASK_H_
#define BMS_MONITOR_TASK_H_

#include "bms_monitor_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "modbus_rtu.h"
#include "jk_bms.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Modbus RTU interface instance (defined in main.c) */
extern struct modbus_rtu_interface_s rtu_interface;

/**
 * @brief Initialise and start the BMS monitoring task.
 *
 * Creates the FreeRTOS data queue, initialises the RS-485 MUX, loads
 * cached serial numbers from NVS, and starts the monitoring task on Core 0.
 *
 * @param[in] interface  Pointer to the Modbus RTU read/write interface.
 * @return ESP_OK on success.
 */
esp_err_t bms_monitor_task_init(struct modbus_rtu_interface_s *interface);

/**
 * @brief Return the JK-BMS device array.
 *
 * @return Pointer to jk_device_t[BMS_BATTERY_COUNT].
 */
jk_device_t *bms_monitor_get_devices(void);

/**
 * @brief Get the FreeRTOS queue handle for battery data.
 *
 * @return QueueHandle_t for bms_queued_data_t items.
 */
QueueHandle_t bms_monitor_get_queue_handle(void);

/**
 * @brief Return the number of batteries successfully read in the last cycle.
 *
 * Thread-safe (volatile read).
 *
 * @return Count in range [0, BMS_BATTERY_COUNT].
 */
uint8_t bms_monitor_get_ok_count(void);

/**
 * @brief Get a copy of system-wide read statistics.
 *
 * @param[out] stats  Pointer to statistics structure to populate.
 */
void bms_monitor_get_stats(bms_system_stats_t *stats);

/** @brief Reset all system statistics to zero. */
void bms_monitor_reset_stats(void);

/**
 * @brief Return true if the BMS monitoring task has started.
 *
 * @return true if running.
 */
bool bms_monitor_task_is_running(void);

/**
 * @brief Check whether the trike is currently in motion.
 *
 * Uses LIS3DHTR hardware interrupt as primary indicator; falls back to
 * battery current draw if the accelerometer is unavailable.
 *
 * @return true if motion is detected.
 */
bool check_trike_motion(void);

/**
 * @brief Get the FreeRTOS task stack high-water mark for the BMS task.
 *
 * @return Minimum free stack in words.
 */
uint32_t bms_monitor_task_get_stack_hwm(void);

/**
 * @brief Set discharge MOSFET state on all BMS batteries.
 *
 * Iterates all BMS_BATTERY_COUNT devices; calls jk_bms_toggle_discharge()
 * with up to 2 retries each.  Charge MOSFET is not touched.
 *
 * @param[in] state  JK_MOSFET_ENABLE or JK_MOSFET_DISABLE.
 * @return true if all succeeded.
 */
bool bms_monitor_set_discharge_all(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif /* BMS_MONITOR_TASK_H_ */