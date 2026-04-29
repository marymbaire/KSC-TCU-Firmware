# KSC TCU V1.1 — Trike Control Unit Firmware

**Platform:** ESP32-S3 · **Framework:** ESP-IDF v5.4.1 · **FW Version:** 1.1.0 · **HW Revision:** V1.0.0

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware Platform](#hardware-platform)
3. [Repository Structure](#repository-structure)
4. [Component Architecture](#component-architecture)
5. [FreeRTOS Tasks](#freertos-tasks)
6. [Data Flow](#data-flow)
7. [Key Data Structures](#key-data-structures)
8. [ThingsBoard Telemetry Fields](#thingsboard-telemetry-fields)
9. [NVS Namespaces](#nvs-namespaces)
10. [MQTT Provisioning Flow](#mqtt-provisioning-flow)
11. [Build System](#build-system)
12. [Known Issues / Review Focus Areas](#known-issues--review-focus-areas)

---

## Overview

The KSC TCU (Trike Control Unit) is a cellular IoT device mounted on electric three-wheel motorcycle trikes in the Songa Trikes fleet. It:

- Monitors **3 × JK-BMS lithium battery packs** via RS-485 Modbus RTU
- Publishes telemetry to **ThingsBoard** (`thingsboard.iot.songa.mobi`) over MQTT via a **Quectel EG915N** LTE modem
- Streams **GPS position** (NMEA sentences) from the modem's debug UART
- Controls **trike power** (relay gate on GPIO40) with motion-aware shutdown
- Measures **pack voltage, current, and temperature** via ADC1
- Detects **motion, tilt, impact, and free-fall** via a LIS3DHTR MEMS accelerometer
- Supports **MQTT Basic provisioning** — new units self-register on ThingsBoard without manual credential entry
- Runs **factory self-tests** at every boot: accelerometer, modem, RS-485 MUX, RGB LEDs, ADC

### V1.0 → V1.1 Changes

| Area | V1.0 (ESP32) | V1.1 (ESP32-S3) |
|------|-------------|-----------------|
| MCU | ESP32 | ESP32-S3 |
| RS-485 routing | 3× GPIO enable pins | MC74HC4052ADG analog MUX (GPIO8/9 select) |
| Current sense | Discrete shunt | INA240A1PWR (gain=20, Vref=1650 mV) |
| Motion detect | None | LIS3DHTR accelerometer (I2C, INT1=GPIO7) |
| GPS | AT+QGPSLOC polling | Continuous NMEA streaming via UART0 |
| LEDs | Single GPIO15 | RGB: Red=GPIO1, Green=GPIO2, Blue=GPIO42 |
| Buzzer | Yes (LEDC) | Removed |
| Button | Yes | Removed |
| Mains detect | Yes (GPIO pin) | Removed |
| MQTT credentials | Compile-time | NVS-backed + auto-provisioning |
| Trike power pin | GPIO27 | GPIO40 |

---

## Hardware Platform

```
ESP32-S3
├── UART0  (GPIO43 TX / GPIO44 RX)  — Quectel debug UART, continuous NMEA
├── UART1  (GPIO12 TX / GPIO13 RX)  — RS-485 half-duplex via MC74HC4052ADG MUX
├── UART2  (GPIO17 TX / GPIO18 RX)  — Quectel main UART (AT commands + MQTT)
├── I2C0   (GPIO47 SCL / GPIO48 SDA) — LIS3DHTR accelerometer (addr 0x18)
├── ADC1
│   ├── CH3 (GPIO4)  — Voltage measurement (÷41 divider, 56–84 V range)
│   ├── CH4 (GPIO5)  — Current measurement (INA240A1PWR, Vref=1650 mV)
│   └── CH5 (GPIO6)  — NTC temperature (10 kΩ B=3950, pull-up R=10 kΩ)
├── GPIO1   — LED Red  (always ON = power indicator)
├── GPIO2   — LED Green (heartbeat)
├── GPIO7   — LIS3DHTR INT1 (motion interrupt, polled)
├── GPIO8   — MUX SEL_A
├── GPIO9   — MUX SEL_B
├── GPIO10  — Quectel RST
├── GPIO11  — Quectel PWR_KEY
├── GPIO15  — Quectel RI (input)
├── GPIO16  — Quectel DTR (output)
├── GPIO40  — Trike power relay gate
└── GPIO42  — LED Blue (RPC/MQTT activity)
```

**Quectel EG915N modem** provides LTE-M/NB-IoT connectivity and GNSS (GPS/GLONASS/BeiDou/Galileo).

---

## Repository Structure

```
KSC_TCU/
├── CMakeLists.txt               # Project root — EXTRA_COMPONENT_DIRS
├── sdkconfig                    # ESP-IDF hardware configuration
├── main/
│   ├── main.c                   # app_main, payload builders, task creation
│   └── CMakeLists.txt
└── components/
    ├── board_config/            # HEADER-ONLY: all pin/constant definitions
    │   └── include/
    │       ├── board_config.h   # GPIO, UART, ADC, circuit parameters
    │       └── version.h        # FW_VERSION_STR, HW_VERSION_STR
    ├── rgb_led/                 # RGB LED alert system
    ├── trike_sensors/           # ADC: voltage, current, temperature
    ├── lis3dhtr/                # LIS3DHTR accelerometer driver
    ├── bms_monitor_types/       # HEADER-ONLY: shared BMS types
    ├── bms_rs485_mux/           # MC74HC4052ADG MUX control
    ├── bms_monitor_task/        # JK-BMS polling task
    ├── power_trike_ctrl/        # Power management + trike relay control
    ├── Quectel_gps/             # NMEA streaming GPS driver
    ├── Quectel_mqtt/            # AT-command MQTT transport + provisioning
    ├── factory_test/            # Boot-time peripheral self-test
    ├── tcu_nvs_creds/           # NVS credential storage
    │
    │   ── Submodules (team-owned, not regenerated) ──
    ├── jk_bms/                  # JK-BMS Modbus RTU register map + reads
    ├── modbus_rtu/              # Generic Modbus RTU framing layer
    ├── bms_common/              # BMS data normalisation (jk_data → common)
    └── jkbms_serial_storage/    # NVS serial-number cache with wear levelling
```

### Header-Only Components

`board_config` and `bms_monitor_types` have no `.c` files. Their `CMakeLists.txt` contains only `INCLUDE_DIRS "include"` with no `SRCS`. Any component that `REQUIRES board_config` gains access to both `board_config.h` and `version.h`.

---

## Component Architecture

### `board_config` (header-only)

Single source of truth for all hardware assignments. No application file may contain raw `GPIO_NUM_xx` literals outside this header. Defines ADC channel constants, voltage divider ratios, INA240 gain, NTC parameters, and all UART/I2C port numbers.

### `rgb_led`

Replaces the V1.0 buzzer/LEDC tone system. A single background FreeRTOS task polls at 50 ms resolution and drives one of eight named alert patterns. Callers call `rgb_led_set_alert(mode)` — they never block. `rgb_led_notify_rpc()` triggers a 5-second blue-blink override for inbound RPC commands without disturbing the base alert state. `rgb_led_set_raw()` is provided for factory test use only.

**Alert modes (priority order):**

| Mode | Pattern | Meaning |
|------|---------|---------|
| `ALERT_BOOT` | All ON | Booting |
| `ALERT_NORMAL` | Green 1 s heartbeat, Red steady | Healthy |
| `ALERT_TRIKE_OFF` | Red steady | Trike unauthorised |
| `ALERT_LOW_SOC_WARN` | Blue steady | Avg SOC < 20% |
| `ALERT_LOW_SOC_CRIT` | Green steady | Avg SOC < 10% |
| `ALERT_NO_BATTERIES` | Red fast blink | No BMS readable |
| `ALERT_VOLTAGE_ANOMALY` | Red+Blue rapid blink | Cross-battery deviation > 5 V |

### `trike_sensors`

Manages ADC1 in IDF v5.x `adc_oneshot` mode with `adc_cali_curve_fitting` calibration (ESP32-S3 supports curve fitting only — line fitting is ESP32/ESP32-S2 only). Reads 64-sample oversampled averages per channel. Shunt resistance and temperature offset are NVS-tunable at runtime via `trike_sensors_calibrate_shunt()` and `trike_sensors_calibrate_current_offset()` without reflashing.

**Circuit parameters (board_config.h):**

- Voltage: R_upper=200 kΩ, R_lower=5 kΩ → scale factor 41.0×, ADC range 56–84 V
- Current: INA240A1PWR gain=20, Vref=1650 mV, shunt=0.00025 Ω (300 A/75 mV external shunt)
- Temperature: NTC 10 kΩ, B=3380 K, pull-up 10 kΩ, Steinhart-Hart beta equation

### `lis3dhtr`

Full LIS3DHTR accelerometer driver over I2C. Configured at 200 Hz ODR, ±16 g full-scale, HR mode, with hardware high-pass filter (cutoff ~1 Hz) on INT1 to remove gravity from motion detection. Both motion detection (INT1_CFG activity interrupt) and impact detection (CLICK_CFG single-click) are active simultaneously on INT1. INT1 is polled (not ISR-driven) from `bms_monitor_task` and `lis3dhtr_is_in_motion()`.

Two runtime modes switchable via `lis3dhtr_configure_mode()`:
- **PRODUCTION** (default): HPF on, gravity removed, outputs show vibration only
- **TEST**: HPF off, gravity component visible — used for bench verification

### `bms_rs485_mux`

Controls the MC74HC4052ADG dual 4-channel analog MUX. Maps battery ID to (SEL_A, SEL_B) pin states with a lookup table. Always disables all channels before enabling a new one to prevent RS-485 bus contention.

| Battery | SEL_A | SEL_B |
|---------|-------|-------|
| 1 | 0 | 0 |
| 2 | 1 | 0 |
| 3 | 0 | 1 |
| Idle | 1 | 1 |

### `bms_monitor_task`

Runs on **Core 0**. Cycles through all 3 batteries sequentially, reads ~14 Modbus registers per battery using the `jk_bms` submodule, and enqueues results into a FreeRTOS queue (`bms_data_queue`, depth 6 = 3 batteries × 2 slots). Motion detection uses `lis3dhtr_is_in_motion()` as primary source, with battery current > 5 A as fallback. Serial number change detection flags battery swaps and forces NVS writes via `jkbms_serial_storage`.

### `power_trike_ctrl`

Merged module combining V1.0's separate `power_management` and `trike_power_ctrl`. Removed: button detection, mains-detect GPIO, backup battery ADC. Added: motion-aware shutdown pipeline with 30-minute safety timeout.

`trike_ctrl_handle_command()` routes trike RPC commands:
- **Value 2 → ON**: immediately asserts GPIO40, saves to NVS, publishes state
- **Value 1 → OFF**: checks motion; if stationary → immediate poweroff; if moving → spawns `trike_wait_for_stop_task` which polls every 5 seconds, forces poweroff after 30 minutes

Power confirmation state is NVS-backed (namespace `trike_ctrl`, key `pwr_conf`) and restored at every boot.

### `Quectel_gps`

GPS changed from V1.0's blocking `AT+QGPSLOC=2` polling to continuous NMEA streaming on UART0. `gps_init()` sends `AT+QGPSCFG="outport","uartdebug"` then `AT+QGPS=1` on the main modem UART, which routes NMEA sentences to the debug UART (GPIO44). A background task parses `$GPRMC` and `$GPGGA` sentences and maintains `gps_state.position`. `gps_get_latest_position()` falls back to `AT+QGPSGNMEA` polling if the streaming cache is empty. Only valid fixes (fix quality ≥ 1, satellites ≥ 3) are returned to the publish task — stale coordinates are never published.

### `Quectel_mqtt`

AT-command MQTT transport over UART2. Key design decisions:

**Thread safety:** `send_at_command()` acquires `uart_modem_mutex` before every AT exchange. A single static `local_response[832]` buffer (BSS, not stack) is used internally — safe because the mutex serialises all callers.

**Response buffer ownership:** Every caller may pass an optional `response_buf` / `response_buf_size` to receive the raw modem response. This eliminates the V1.0 global `urcbuffer`/`gpsbuffer` shared-state race condition.

**RPC acknowledgement:** `read_buffered_messages()` now builds and publishes an `RpcResponse` to `v1/devices/me/rpc/response/{requestId}` after executing each command, closing the ThingsBoard RPC loop so the dashboard does not show commands as timed out.

**Provisioning:** `tcu_load_or_provision_creds()` is called at startup before any normal MQTT connection. It loads credentials from NVS (`tcu_creds` namespace) or runs the full ThingsBoard MQTT Basic provisioning flow (connect as `"provision"` → publish to `/provision/request` → wait for `/provision/response` URC → parse → save to NVS → restart).

**Re-provisioning triggers:**
- MQTT auth return code 5 (device deleted from ThingsBoard)
- RPC method `"TRSW"` (trike swap command)
- `tcu_force_reprovision()` erases NVS credentials, next boot re-provisions

### `factory_test`

Runs at every boot in `app_main()` before normal operation begins. Results print to USB CDC console as `[PASS]`/`[FAIL]` lines. Tests:

1. **LIS3DHTR** — WHO_AM_I check (expect 0x33) + live accelerometer read
2. **Quectel modem** — 8-stage AT sequence: AT / ATI / ATE0 / AT+CGMI / AT+GSN (IMEI) / AT+CSQ (signal) / AT+CREG (registration, up to 180 s wait) / AT+CPIN (SIM). Up to 3 full power-cycle retries.
3. **RS-485 MUX** — Transmits 5× `"Hello World\r\n"` on each of 3 battery buses; tester observes TX LEDs
4. **RGB LEDs** — Cycles RED → GREEN → BLUE → WHITE at 200 ms each (visual, auto-pass)
5. **ADC** — Reads voltage/current/temperature; checks ranges (V: <1 V or 40–100 V; I: ±10 A; T: −40 to +120 °C)

### `tcu_nvs_creds`

Manages the `tcu_creds` NVS namespace. Stores `prov_done` flag, `username`, `password`, `client_id`, `device_name`. The `prov_done` flag is the single source of truth — credentials present without this flag are treated as invalid. `tcu_nvs_repair()` attempts namespace-level erase first, then full partition erase on failure.

---

## FreeRTOS Tasks

| Task name | Function | Core | Priority | Stack | Component |
|-----------|----------|------|----------|-------|-----------|
| `app_main` | Hardware init, task creation | 0 | 1 | default | `main` |
| `bms_monitor` | JK-BMS cyclic read, MUX control | 0 | 5 | 4096 w | `bms_monitor_task` |
| `mqtt_publish` | Periodic telemetry publish, keepalive, power events | 1 | 5 | 6144 w | `main` |
| `urc_proc` | MQTT URC dequeue → RPC dispatch | 1 | 5 | 4096 w | `main` |
| `gps_nmea` | UART0 NMEA sentence parser | 0 | 4 | 3072 w | `Quectel_gps` |
| `rgb_led` | LED blink pattern driver, 50 ms tick | 0 | 2 | 2048 w | `rgb_led` |
| `power_monitor` | Modem health check, brownout recovery | 1 | 6 | 4096 w | `power_trike_ctrl` |
| `stats_monitor` | BMS statistics logging every 30 s | 1 | 3 | 3072 w | `main` |
| `diag_log` | 1 Hz bench diagnostic logging (remove pre-production) | 0 | 2 | 4072 w | `main` |
| `trike_wait` | Motion-aware delayed poweroff (spawned on demand) | 1 | 4 | 3072 w | `power_trike_ctrl` |

### Inter-task Communication

```
bms_monitor  ──→  bms_data_queue (depth 6, bms_queued_data_t)  ──→  mqtt_publish
                                                                    └──→  urc_proc
modem UART  ──→  urc_notification_queue (depth 10, urc_notification_t)  ──→  urc_proc
power events ──→  power_event_queue (depth 5, power_event_t)  ──→  mqtt_publish
gps_nmea ──→  gps_state.position (mutex-protected struct)  ──→  mqtt_publish (via getter)
```

**Shared mutex:** `uart_modem_mutex` (created in `Quectel_mqtt`, extern in `power_trike_ctrl`) serialises all UART2 AT command traffic across `mqtt_publish`, `urc_proc`, `power_monitor`, and `Quectel_gps` tasks.

---

## Data Flow

```
┌──────────────┐   Modbus RTU    ┌────────────┐   FreeRTOS Queue   ┌──────────────────┐
│  JK-BMS ×3   │ ◄──────────── ►│ bms_monitor│ ─────────────────► │  mqtt_publish    │
│  (RS-485)    │   via MUX      │   _task    │                     │  (Core 1)        │
└──────────────┘                └────────────┘                     │                  │
                                                                    │  build JSON      │
┌──────────────┐   NMEA UART0   ┌────────────┐   mutex+struct      │  payloads        │
│ Quectel GPS  │ ─────────────► │  gps_nmea  │ ─────────────────► │                  │
│  (debug UART)│                │   (task)   │                     │  AT+QMTPUBEX     │
└──────────────┘                └────────────┘                     │  ──────────────► │
                                                                    │  ThingsBoard     │
┌──────────────┐   ADC1         ┌────────────┐   direct call       │  via Quectel     │
│  V/I/T ADC   │ ─────────────► │  trike_    │ ─────────────────► │  EG915N MQTT     │
│  sensors     │                │  sensors   │                     └──────────────────┘
└──────────────┘                └────────────┘

┌──────────────┐   I2C / GPIO7  ┌────────────┐   direct call       ┌──────────────────┐
│  LIS3DHTR    │ ◄──────────── ►│  lis3dhtr  │ ─────────────────► │  bms_monitor     │
│  accel       │                │  (driver)  │   is_in_motion()    │  (motion detect) │
└──────────────┘                └────────────┘                     └──────────────────┘
                                      │
                                      │ direct call
                                      ▼
                               ┌────────────────┐
                               │  power_trike   │
                               │  _ctrl         │
                               │ (motion-aware  │
                               │  shutdown)     │
                               └────────────────┘
```

---

## Key Data Structures

### `bms_queued_data_t` (`bms_monitor_types.h`)

The primary data carrier between the BMS monitor task and the MQTT publish task. One instance per battery per read cycle, enqueued in `bms_data_queue`.

```c
typedef struct {
    bms_battery_id_t  battery_id;        // 0–2
    bms_read_status_t status;            // OK / TIMEOUT / CRC_ERROR / NO_DATA
    uint32_t          timestamp;         // System tick (ms)
    uint32_t          batt_v;            // Pack voltage (mV)
    int32_t           batt_i;            // Pack current (mA), +ve = discharge
    int32_t           batt_power;        // Pack power (mW)
    int32_t           remaining_capacity;// Remaining capacity (mAh)
    uint32_t          cell_count;
    uint32_t          cells_present;
    uint16_t          cells_diff;        // Max cell voltage difference (mV)
    uint8_t           soc;               // State of charge (%)
    uint8_t           soh;               // State of health (%)
    uint8_t           charge_stat;       // Charge MOSFET state
    uint8_t           discharge_stat;    // Discharge MOSFET state
    uint32_t          charge_cycles;
    uint32_t          alarms;            // JK-BMS alarm bitmask
    char              model_no[32];
    char              serial_no[32];
} bms_queued_data_t;
```

### `lis3dhtr_motion_t` (`lis3dhtr.h`)

Output of `lis3dhtr_read_motion()`. All fields published to ThingsBoard except `roll_deg` and `pitch_deg` which are computed but retained for future use.

```c
typedef struct {
    float roll_deg;          // Roll (not published)
    float pitch_deg;         // Pitch (not published)
    float accel_mag_mg;      // Vector magnitude = sqrt(x²+y²+z²) mg
    float accel_rms_mg;      // RMS = sqrt((x²+y²+z²)/3) mg — vibration scalar
    float x_mg;              // X-axis acceleration mg  → P31
    float y_mg;              // Y-axis acceleration mg  → P32
    float z_mg;              // Z-axis acceleration mg  → P33
    bool  in_motion;         // Hardware INT1 activity latch → P36
    bool  impact_detected;   // CLICK_SRC single-click latch
    bool  free_fall;         // |a| < 350 mg
} lis3dhtr_motion_t;
```

### `trike_sensor_data_t` (`trike_sensors.h`)

Output of `trike_sensors_read_all()`. Both engineering-unit and raw ADC values are carried so the backend can recompute with updated calibration parameters if circuit constants change.

```c
typedef struct {
    float    voltage_v;              // Pack voltage (V)          → P28
    float    current_a;              // Pack current (A)          → P29
    float    temperature_c;          // Temperature (°C)          → P30
    bool     valid;
    int      raw_voltage;            // Raw 12-bit ADC count
    int      raw_current;
    int      raw_temp;
    float    adc_mv_voltage;         // Calibrated mV
    float    adc_mv_current;
    float    adc_mv_temp;
    float    current_offset_applied_a;
} trike_sensor_data_t;
```

### `power_event_t` (`power_trike_ctrl.h`)

Posted to `power_event_queue` by the power monitor task. Currently only `POWER_EVENT_MODEM_BROWNOUT` is in use; the V1.0 `MAINS_LOST`/`MAINS_RESTORED` events have been removed with the mains-detect pin.

```c
typedef struct {
    power_event_type_t type;       // POWER_EVENT_MODEM_BROWNOUT
    uint32_t           timestamp_ms;
} power_event_t;
```

### `urc_notification_t` (`Quectel_mqtt.h`)

Posted to `urc_notification_queue` by `check_mqtt_urc()` when a `+QMTRECV:` unsolicited result code is detected on the modem UART.

```c
typedef struct {
    uint8_t  client_idx;    // MQTT client index (always 0)
    uint8_t  recv_id;       // Receive slot index → passed to read_buffered_messages()
    uint32_t timestamp_ms;  // Tick when URC was seen (for latency monitoring)
} urc_notification_t;
```

### `tcu_mqtt_creds_t` (`tcu_nvs_creds.h`)

Holds the per-device MQTT credentials obtained during ThingsBoard MQTT Basic provisioning. Persisted in NVS namespace `tcu_creds`.

```c
typedef struct {
    char username[64];
    char password[64];
    char client_id[64];
    char device_name[64];
} tcu_mqtt_creds_t;
```

---

## ThingsBoard Telemetry Fields

All fields are published to topic `v1/devices/trikes/telemetry`.

### Per-battery fields (prefix B1\_, B2\_, B3\_)

| Field | Unit | Description |
|-------|------|-------------|
| `Bx_P12` | V | Pack voltage |
| `Bx_P13` | A | Pack current |
| `Bx_P14` | % | State of charge |
| `Bx_P15` | % | State of health |
| `Bx_P16` | — | Charge cycle count |
| `Bx_P17` | — | Read status code |
| `Bx_P18` | — | Charge MOSFET state |
| `Bx_P19` | — | Discharge MOSFET state |
| `Bx_P20` | — | BMS serial number string |
| `Bx_P21` | W | Pack power |
| `Bx_P22` | mV | Max cell voltage difference |
| `Bx_P23` | — | Alarm bitmask |
| `Bx_P24` | — | Cell count |
| `Bx_P25` | Ah | Remaining capacity |
| `Bx_P26` | 0/1 | Cross-battery voltage anomaly flag |

### Metadata / device fields

| Field | Unit | Description |
|-------|------|-------------|
| `"1"` | 0/1 | Mains present (always 1 on V1.1) |
| `"2"` | — | Device serial (MAC hex string) |
| `"3"` | — | Valid battery count |
| `"4"` | ° | GPS latitude |
| `"5"` | ° | GPS longitude |
| `"6"` | m | GPS altitude |
| `"7"` | — | GPS satellites in use |
| `"8"` | — | GPS fix quality |
| `"9"` | bool | Trike power resp_cmd (queued poweroff flag) |
| `"10"` | bool | Power confirmation (actual trike state) |
| `"11"` | — | Reset reason string (omitted on clean power-on) |
| `"27"` | % | Average battery SOC (all 3 batteries) |
| `P28` | V | Trike ADC voltage |
| `P29` | A | Trike ADC current |
| `P30` | °C | Powertrain temperature |
| `P31` | mg | Accelerometer X-axis |
| `P32` | mg | Accelerometer Y-axis |
| `P33` | mg | Accelerometer Z-axis |
| `P34` | mg | Accelerometer RMS |
| `P35` | mg | Accelerometer magnitude |
| `P36` | 0/1 | Motion state |
| `P37` | dBm | GSM/LTE signal strength |
| `fw` | — | Firmware version string |
| `hw` | — | Hardware revision string |

---

## NVS Namespaces

| Namespace | Component | Keys | Purpose |
|-----------|-----------|------|---------|
| `tcu_creds` | `tcu_nvs_creds` | `prov_done`, `username`, `password`, `client_id`, `device_name` | MQTT provisioning credentials |
| `trike_ctrl` | `power_trike_ctrl` | `pwr_conf` | Trike power state (survives reboot) |
| `bms_serials` | `jkbms_serial_storage` | `bat_sn_0..2`, `bat_cnt_0..2` | Battery serial numbers with write counter (wear levelling: write every 100 changes) |
| `trike_sens` | `trike_sensors` | `shunt_ohms`, `temp_offset`, `curr_offset` | Field-tunable ADC calibration constants |

---

## MQTT Provisioning Flow

```
Boot
 │
 ├─ tcu_nvs_is_provisioned() == true?
 │     YES → tcu_nvs_load_creds() → populate runtime buffers → normal MQTT connect
 │     NO  ──────────────────────────────────────────────────────────────────────┐
 │                                                                                ▼
 │                                              Connect as "provision" / "" (TB spec)
 │                                              Subscribe /provision/response
 │                                              Publish /provision/request:
 │                                              { deviceName, provisionDeviceKey,
 │                                                credentialsType: "MQTT_BASIC",
 │                                                username, password, clientId }
 │                                                       │
 │                                              Wait for +QMTRECV URC (15 s timeout)
 │                                                       │
 │                                              parse /provision/response:
 │                                              { status: "SUCCESS",
 │                                                credentialsValue: { userName,
 │                                                  password, clientId } }
 │                                                       │
 │                                              tcu_nvs_save_creds() → esp_restart()
 │                                                       │
 └──────────────────────────────────────────────────────┘
                                   Next boot: normal MQTT connect with saved creds
```

**Re-provisioning:** Sending RPC method `"TRSW"` or receiving MQTT auth error (return code 5) calls `tcu_force_reprovision()` which erases the `tcu_creds` NVS namespace. The device restarts and re-runs the provisioning flow.

---

## Build System

**IDF version:** v5.4.1 (xtensa-esp-elf GCC 14.2.0)

Each custom component has its own `CMakeLists.txt` following the `idf_component_register()` pattern. The project root `CMakeLists.txt` uses `EXTRA_COMPONENT_DIRS` with one entry per component directory.

**IDF v5.x migration notes applied in this codebase:**

| V4.x name | V5.x replacement |
|-----------|-----------------|
| `esp_adc_cal` component | `esp_adc` component |
| `driver/adc.h` | `esp_adc/adc_oneshot.h` + `esp_adc/adc_cali.h` |
| `ADC1_CHANNEL_x` constants | `ADC_CHANNEL_x` |
| `adc_cali_line_fitting` | Removed on ESP32-S3; use `adc_cali_curve_fitting` only |
| `esp_task_wdt` component | Absorbed into `esp_system` |
| `esp_adc_cal_characterize()` | `adc_cali_create_scheme_curve_fitting()` |

---

## Known Issues / Review Focus Areas

The following items are flagged for the code review team.

### 1. `diag_log_task` — remove before production
`app_main()` spawns a 1 Hz diagnostic logging task that calls `lis3dhtr_read_motion()`, `trike_sensors_read_all()`, `gps_get_latest_position()`, and `get_gsm_signal_quality()` in a tight loop. This adds significant AT command traffic to the modem UART. Remove or gate behind `#ifdef CONFIG_TCU_DIAG_MODE` before field deployment.

### 2. `bms_led_status_task` — disabled
`bms_monitor_task_init()` has the LED status subtask creation commented out. The current behaviour is that no LED feedback is given for BMS-level faults unless the MQTT publish task detects `ok_count == 0`. Review whether this is intentional.

### 3. Watchdog disabled
`esp_task_wdt_reconfigure()` and `esp_task_wdt_add()`/`esp_task_wdt_reset()` calls are commented out in `app_main()` and `mqtt_publish_task`. The production watchdog timeout should be re-enabled (300 s) before field deployment.

### 4. Hardcoded credentials in `tcu_load_or_provision_creds()`
Lines 1–10 of `tcu_load_or_provision_creds()` short-circuit the NVS/provisioning flow with hardcoded test credentials (`tcutest2`/`tcutest2`/`569745`) and an early `return true`. This must be removed before production — the `return true` should be deleted and the `tcu_nvs_is_provisioned()` / `tcu_nvs_load_creds()` path should be the active code path.

### 5. `create_metadata_payload()` signature mismatch
`Quectel_mqtt.h` declares `char *create_metadata_payload()` (no parameters) but `Quectel_mqtt.c` calls `create_metadata_payload(false)` in `mqtt_pubclient_battery()`. In C, an empty parameter list `()` means "unspecified arguments" — this compiles but is technically undefined behaviour. The declaration should be `char *create_metadata_payload(void)` and all call sites updated consistently.

### 6. Circular dependency: `Quectel_mqtt` ↔ `power_trike_ctrl`
`Quectel_mqtt.c` calls `trike_ctrl_handle_command()` and `trike_ctrl_get_power_confirmation()` from `power_trike_ctrl`. `power_trike_ctrl.c` calls `mqtt_pubclient_status()` and `open_mqtts_with_creds()` from `Quectel_mqtt`. This bidirectional dependency is workable in ESP-IDF CMake but means neither component can be unit-tested in isolation. Consider introducing a thin event queue or callback interface to decouple them.

### 7. `mqtt_pubclient_battery()` calls `create_metadata_payload(false)`
The `false` argument is a leftover from when `create_metadata_payload` accepted a `bool include_shutdown_flag` parameter. Since that parameter was removed from the implementation, this is currently calling a no-arg function with an argument. Align declaration, implementation, and all call sites.

### 8. `power_mgmt_recover_modem()` uses undefined symbols
`power_trike_ctrl.c` calls `open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password)` directly. These are `Quectel_mqtt.c` globals — they are correctly `extern` declared via the header, but the circular include means the linker resolves them correctly only because both `.c` files are in the same final binary. This is fragile if either component is ever moved to a separate library.

### 9. `bms_monitor_task` stack size
The BMS monitor task stack is 4096 words (16 KB). With the `READ_REG` macro expansion and `jk_data_t` on the stack, the high-water mark should be checked after a full cycle in field conditions. Stack depth < 256 words at runtime triggers a log warning — verify this warning has not been observed in testing.

### 10. GPS UART0 / console conflict
`board_config.h` assigns UART0 (GPIO43/44) to GPS NMEA streaming. UART0 is also the default ESP-IDF console UART. If `sdkconfig` has `CONFIG_ESP_CONSOLE_UART_NUM=0`, ESP-IDF will write boot logs and `printf` output to GPIO43/44 before `gps_init()` installs the driver. Verify `sdkconfig` sets `CONFIG_ESP_CONSOLE_USB_CDC=y` or `CONFIG_ESP_CONSOLE_UART_NONE=y` so UART0 is fully reserved for GPS.