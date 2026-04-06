/**
 * @file    ecu_config.h
 * @brief   RP2350 EFI – Hardware pin map & engine calibration constants
 *
 * Board:   RB-RP2354A (Curio flight-controller variant repurposed for EFI demo)
 * Target:  Raspberry Pi RP2350 (dual Cortex-M33, 150 MHz)
 * SDK:     pico-sdk 2.x
 *
 * Dual-core assignment
 *   Core 0 → deterministic control loop  (fuel, ignition, ISR)   @ 2 kHz
 *   Core 1 → housekeeping                (serial tuning, CAN TX)  @ 100 Hz
 */

#pragma once

#include <stdint.h>

/* ─────────────────────────────────────────────────────────────
 *  GPIO / Pin Assignments
 * ───────────────────────────────────────────────────────────── */

// Crank Position Sensor (60-2 VR / Hall – feeds PIO SM0)
#define PIN_CKP             2   // GP2  – digital pulse input to PIO

// Cam Position Sensor   (1-tooth per rev – feeds PIO SM1)
#define PIN_CMP             3   // GP3  – digital pulse input to PIO

// Injector driver outputs (open-drain MOSFET gate)
#define PIN_INJ_1           6   // GP6
#define PIN_INJ_2           7   // GP7
#define PIN_INJ_3           8   // GP8
#define PIN_INJ_4           9   // GP9

// Ignition coil drivers (active-high, logic-level IGBT)
#define PIN_IGN_1           10  // GP10
#define PIN_IGN_2           11  // GP11
#define PIN_IGN_3           12  // GP12
#define PIN_IGN_4           13  // GP13

// Idle Air Control (stepper coils A/B or PWM IAC valve)
#define PIN_IAC_PWM         14  // GP14 – PWM slice 7A

// Fuel pump relay
#define PIN_FUEL_PUMP       15  // GP15

// ADC inputs (RP2350 ADC channels 0-4 → GP26-GP29 + internal)
#define PIN_ADC_MAP         26  // GP26 – ADC0  manifold pressure
#define PIN_ADC_TPS         27  // GP27 – ADC1  throttle position
#define PIN_ADC_IAT         28  // GP28 – ADC2  intake air temperature
#define PIN_ADC_CLT         29  // GP29 – ADC3  coolant temperature
// GP_ADC4 = internal temperature sensor (not used for EFI here)

// Wideband O2 (analog 0-5 V via divider → 0-3.3 V → ADC)
#define PIN_ADC_O2          26  // NOTE: mux with MAP if single ADC – in real HW use separate channel
                                //       Here we assume MUX sequence: MAP→TPS→IAT→CLT→O2 @ 1 kHz each

// UART1 – serial tuning link (TunerStudio / custom protocol)
#define PIN_UART_TX         4   // GP4
#define PIN_UART_RX         5   // GP5
#define UART_BAUD           115200

// SPI0 – optional external MAP sensor (e.g. MS5611 barometric)
#define PIN_SPI_SCK         18
#define PIN_SPI_MOSI        19
#define PIN_SPI_MISO        16
#define PIN_SPI_CS_MAP      17


/* ─────────────────────────────────────────────────────────────
 *  Engine Configuration
 * ───────────────────────────────────────────────────────────── */

#define ENGINE_CYL          4           // number of cylinders
#define ENGINE_STROKE       4           // 4-stroke
#define ENGINE_DISPLACEMENT_CC 1600.0f  // total displacement [cc]

// Crank trigger wheel
#define TRIGGER_TEETH_TOTAL  60         // 60-2 wheel
#define TRIGGER_TEETH_MISSING 2         // missing teeth for sync

// Injector
#define INJ_FLOW_RATE_CC_MIN  240.0f    // cc/min @ 3.0 bar
#define INJ_DEAD_TIME_US      750.0f    // injector latency [µs] @ 14 V
#define FUEL_PRESSURE_BAR     3.0f      // assumed constant rail pressure

// Ignition dwell
#define IGN_DWELL_MS          3.5f      // coil charge time [ms]
#define IGN_MIN_ADVANCE_DEG   0.0f      // TDC
#define IGN_MAX_ADVANCE_DEG   40.0f     // max safe advance

// ADC reference
#define ADC_VREF              3.3f
#define ADC_RESOLUTION        4096.0f   // 12-bit

// Stoichiometric AFR (gasoline)
#define STOICH_AFR            14.7f

// Timing
#define CONTROL_LOOP_HZ       2000      // Core 0 loop rate [Hz]
#define COMMS_LOOP_HZ         100       // Core 1 loop rate [Hz]
#define ADC_SAMPLE_HZ         5000      // DMA ADC round-robin rate


/* ─────────────────────────────────────────────────────────────
 *  Shared data structure (Core 0 writes, Core 1 reads)
 *  Protected by a seqlock (sequence counter + DMB)
 * ───────────────────────────────────────────────────────────── */

#include <pico/sync.h>

typedef struct __attribute__((packed)) {
    // Seqlock counter – Core 1 checks even/odd to detect torn reads
    volatile uint32_t seq;

    // Raw sensor readings (ADC counts, 12-bit)
    uint16_t  adc_map_raw;
    uint16_t  adc_tps_raw;
    uint16_t  adc_iat_raw;
    uint16_t  adc_clt_raw;
    uint16_t  adc_o2_raw;

    // Kalman-filtered physical quantities
    float     map_kpa;          // estimated manifold pressure [kPa]
    float     map_kpa_dot;      // dP/dt [kPa/s] – from KF state
    float     iat_degC;         // estimated intake air temp [°C]
    float     clt_degC;         // coolant temp [°C]
    float     tps_pct;          // throttle position [0-100 %]
    float     lambda;           // measured λ from O2 sensor
    float     air_mass_mg;      // estimated air mass per cycle [mg]

    // Crank / speed
    float     rpm;              // engine speed [RPM]
    uint16_t  crank_angle_deg;  // current crank angle [0-719]
    bool      engine_sync;      // true when fully synced to trigger wheel

    // EFI outputs
    float     ipw_us;           // injector pulse width [µs]
    float     ign_advance_deg;  // ignition advance [° BTDC]
    float     lambda_trim;      // closed-loop O2 correction [-0.25 … +0.25]
    float     iac_duty;         // idle air control [0-100 %]
} EcuState_t;

extern volatile EcuState_t g_ecu;
