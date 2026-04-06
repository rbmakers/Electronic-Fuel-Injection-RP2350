/**
 * @file    main.cpp
 * @brief   RP2350 EFI – Dual-core main entry point
 *
 * ┌────────────────────────────────────────────────────────────┐
 * │ Core 0 – Deterministic Control Loop @ 2 kHz               │
 * │                                                            │
 * │  1. Read ADC DMA buffer  → SensorReadings_t               │
 * │  2. Update MAP Kalman Filter (pressure + rate)             │
 * │  3. Update IAT Kalman Filter                               │
 * │  4. Run fuel_calc_run() → FuelOutput_t                     │
 * │  5. Schedule injector pulses (PIO SM or hardware timer)    │
 * │  6. Update IAC PID                                         │
 * │  7. Publish results → g_ecu via seqlock                    │
 * └────────────────────────────────────────────────────────────┘
 *
 * ┌────────────────────────────────────────────────────────────┐
 * │ Core 1 – Housekeeping @ 100 Hz                             │
 * │                                                            │
 * │  1. Read g_ecu snapshot (seqlock)                          │
 * │  2. Serial TX to tuning software (CSV or binary protocol)  │
 * │  3. Receive serial RX commands (VE/AFR table updates)      │
 * │  4. Optional: CAN TX frame for dashboard                   │
 * └────────────────────────────────────────────────────────────┘
 *
 * Seqlock protocol
 * ─────────────────
 *  Writer (Core 0):  seq++ (now odd) → write data → seq++ (even) → DMB
 *  Reader (Core 1):  repeat { s = seq; read; DMB } until (s == seq && s even)
 */

#include "ecu_config.h"
#include "drivers/crank_sensor.h"
#include "drivers/adc_sensors.h"
#include "fusion/kalman_filter.h"
#include "efi/fuel_calc.h"
#include "efi/pid.h"

// Pull PID implementation into this translation unit
#define PID_IMPL
#include "efi/pid.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

#include <stdio.h>
#include <string.h>

// ── Global shared state ───────────────────────────────────────
volatile EcuState_t g_ecu = {};

// ── Module-level instances ────────────────────────────────────
static MapKalman_t  s_map_kf;
static IatKalman_t  s_iat_kf;
static PID_t        s_iac_pid;

// ── TPS derivative (simple finite difference) ─────────────────
static float s_tps_prev  = 0.0f;
static float s_tps_dot   = 0.0f;   // [%/s]

// ── Seqlock helpers ───────────────────────────────────────────
static inline void seqlock_write_begin(void) {
    g_ecu.seq++;
    __dmb();            // ARM DMB – ensure seq write is visible before data writes
}
static inline void seqlock_write_end(void) {
    __dmb();
    g_ecu.seq++;        // now even again
}

// ── Injector scheduling ───────────────────────────────────────
/**
 * Fire injector for cylinder `cyl` with pulse width `ipw_us`.
 * In production this would be scheduled via the PIO injector_pulse
 * program triggered at the correct crank angle.
 * For this demo we use a direct GPIO pulse via hardware alarm.
 */
static void schedule_injection(uint cyl_gpio, float ipw_us) {
    if (ipw_us < 100.0f) return;   // skip degenerate pulse
    gpio_put(cyl_gpio, 1);
    busy_wait_us((uint32_t)ipw_us);
    gpio_put(cyl_gpio, 0);
    // NOTE: In real firmware replace busy_wait with a timer alarm callback
    //       or the PIO injector_pulse state machine to avoid blocking Core 0.
}

// ── IAC PWM output ────────────────────────────────────────────
static void iac_set_duty(float duty_pct) {
    uint slice = pwm_gpio_to_slice_num(PIN_IAC_PWM);
    uint16_t level = (uint16_t)(duty_pct / 100.0f * 65535.0f);
    pwm_set_gpio_level(PIN_IAC_PWM, level);
}

// ── UART serial output ────────────────────────────────────────
static void uart_send_csv_snapshot(const volatile EcuState_t *e) {
    // CSV: rpm, map_kpa, iat_degC, clt_degC, tps_pct, lambda,
    //      ipw_us, ign_adv, lambda_trim, air_mass_mg
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "%.0f,%.2f,%.1f,%.1f,%.1f,%.3f,%.1f,%.1f,%.3f,%.2f\r\n",
        (double)e->rpm,
        (double)e->map_kpa,
        (double)e->iat_degC,
        (double)e->clt_degC,
        (double)e->tps_pct,
        (double)e->lambda,
        (double)e->ipw_us,
        (double)e->ign_advance_deg,
        (double)e->lambda_trim,
        (double)e->air_mass_mg);
    uart_puts(uart1, buf);
}

// ─────────────────────────────────────────────────────────────
//  CORE 1 ENTRY – Housekeeping / Communications
// ─────────────────────────────────────────────────────────────
static void core1_entry(void) {
    // Init UART1 for serial tuning link
    uart_init(uart1, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    const uint32_t period_us = 1000000 / COMMS_LOOP_HZ;  // 10 000 µs @ 100 Hz

    while (true) {
        uint64_t tick_start = time_us_64();

        // ── Seqlock read ─────────────────────────────────────
        EcuState_t snap;
        uint32_t s0, s1;
        do {
            s0 = g_ecu.seq;
            __dmb();
            memcpy((void*)&snap, (const void*)&g_ecu, sizeof(snap));
            __dmb();
            s1 = g_ecu.seq;
        } while (s0 != s1 || (s0 & 1u));   // retry if odd (write in progress) or torn

        // ── Serial output ─────────────────────────────────────
        uart_send_csv_snapshot(&snap);

        // ── Receive tuning commands (simple: "VE,r,c,val\n") ──
        while (uart_is_readable(uart1)) {
            // In a full implementation, accumulate into a line buffer
            // and parse VE/AFR table updates here.
            (void)uart_getc(uart1);  // placeholder: discard
        }

        // ── Maintain loop timing ──────────────────────────────
        uint64_t elapsed = time_us_64() - tick_start;
        if (elapsed < period_us) {
            sleep_us(period_us - elapsed);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 SETUP
// ─────────────────────────────────────────────────────────────
static void hardware_init(void) {
    stdio_init_all();

    // Injector GPIO outputs
    const uint inj_pins[] = { PIN_INJ_1, PIN_INJ_2, PIN_INJ_3, PIN_INJ_4 };
    for (int i = 0; i < 4; i++) {
        gpio_init(inj_pins[i]);
        gpio_set_dir(inj_pins[i], GPIO_OUT);
        gpio_put(inj_pins[i], 0);
    }

    // Fuel pump relay
    gpio_init(PIN_FUEL_PUMP);
    gpio_set_dir(PIN_FUEL_PUMP, GPIO_OUT);
    gpio_put(PIN_FUEL_PUMP, 0);

    // IAC PWM (50 Hz, 16-bit)
    gpio_set_function(PIN_IAC_PWM, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_IAC_PWM);
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, 38.15f);   // 150 MHz / 38.15 / 65536 ≈ 50 Hz
    pwm_init(slice, &pwm_cfg, true);
    iac_set_duty(20.0f);  // safe starting position

    // ADC + DMA
    adc_sensors_init();

    // PIO crank sensor
    crank_sensor_init();
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 MAIN – Control Loop @ 2 kHz
// ─────────────────────────────────────────────────────────────
int main(void) {
    hardware_init();

    // ── Module init ───────────────────────────────────────────
    fuel_calc_init();

    // Seed KF from a first ADC reading
    SensorReadings_t first_read;
    adc_sensors_read(&first_read);
    map_kf_init(&s_map_kf, first_read.map_kpa);
    iat_kf_init(&s_iat_kf, first_read.iat_degC);

    // IAC PID: target = 800 RPM, output = duty %
    // Kp=0.05, Ki=0.01, Kd=0.005, output clamp 5-80%, dt=0.5ms
    pid_init(&s_iac_pid, 0.05f, 0.01f, 0.005f, 5.0f, 80.0f,
             1.0f / CONTROL_LOOP_HZ, 0.7f);

    // Prime fuel pump for 2 s before cranking
    gpio_put(PIN_FUEL_PUMP, 1);
    sleep_ms(2000);

    // Start Core 1
    multicore_launch_core1(core1_entry);

    // ── Main control loop timing ──────────────────────────────
    const uint32_t period_us = 1000000 / CONTROL_LOOP_HZ;   // 500 µs @ 2 kHz
    const uint inj_pins[]    = { PIN_INJ_1, PIN_INJ_2, PIN_INJ_3, PIN_INJ_4 };
    static const float INJ_FIRE_ANGLE[] = { 0, 180, 360, 540 };  // BTDC crank angles

    while (true) {
        uint64_t tick_start = time_us_64();

        // ── 1. ADC snapshot ───────────────────────────────────
        SensorReadings_t sens;
        adc_sensors_read(&sens);

        // ── 2. Kalman Filter updates ──────────────────────────
        uint64_t now_us = time_us_64();
        map_kf_update(&s_map_kf, sens.map_kpa,  now_us);
        iat_kf_update(&s_iat_kf, sens.iat_degC);

        float map_est    = map_kf_get_pressure(&s_map_kf);
        float map_dot    = map_kf_get_rate    (&s_map_kf);
        float iat_est    = iat_kf_get_temp    (&s_iat_kf);

        // ── 3. TPS derivative (finite diff) ──────────────────
        s_tps_dot  = (sens.tps_pct - s_tps_prev) * CONTROL_LOOP_HZ; // %/s
        s_tps_prev = sens.tps_pct;

        // ── 4. Crank state ────────────────────────────────────
        float rpm         = crank_get_rpm();
        bool  synced      = crank_is_synced();
        uint16_t angle_10 = crank_get_angle_tdeg();   // tenths of degree

        // ── 5. Fuel calculation ───────────────────────────────
        FuelInput_t  fi = {};
        FuelOutput_t fo = {};

        fi.rpm         = rpm;
        fi.map_kpa     = map_est;
        fi.map_kpa_dot = map_dot;
        fi.iat_degC    = iat_est;
        fi.clt_degC    = sens.clt_degC;
        fi.tps_pct     = sens.tps_pct;
        fi.tps_dot     = s_tps_dot;
        fi.lambda_meas = sens.lambda;
        fi.closed_loop = (sens.clt_degC > 70.0f) && synced && (rpm > 500.0f);

        if (synced && rpm > 200.0f) {
            fuel_calc_run(&fi, &fo);
        }

        // ── 6. Inject at correct crank angle ─────────────────
        // Simplified: fire all injectors simultaneously (batch injection)
        // In full sequential code this is triggered by CKP ISR at the
        // per-cylinder firing angle.
        if (synced && fo.ipw_final_us > 300.0f) {
            for (int i = 0; i < ENGINE_CYL; i++) {
                // TODO: Check (angle_10 / 10) ≈ INJ_FIRE_ANGLE[i] within ±5°
                // For demo: fire cylinder 1 only when angle ≈ 0° (TDC)
                if (i == 0 && angle_10 < 50) {   // within 5° of TDC
                    schedule_injection(inj_pins[i], fo.ipw_final_us);
                }
            }
        }

        // ── 7. IAC PID (idle control) ─────────────────────────
        float iac_duty = 20.0f;
        if (synced && sens.tps_pct < 2.0f) {   // only at closed throttle
            iac_duty = pid_update(&s_iac_pid, 800.0f, rpm);
        } else {
            pid_reset(&s_iac_pid);  // prevent integrator windup off-idle
            iac_duty = 20.0f;
        }
        iac_set_duty(iac_duty);

        // ── 8. Publish to shared state (seqlock) ──────────────
        seqlock_write_begin();

        g_ecu.map_kpa        = map_est;
        g_ecu.map_kpa_dot    = map_dot;
        g_ecu.iat_degC       = iat_est;
        g_ecu.clt_degC       = sens.clt_degC;
        g_ecu.tps_pct        = sens.tps_pct;
        g_ecu.lambda         = sens.lambda;
        g_ecu.rpm            = rpm;
        g_ecu.crank_angle_deg= angle_10 / 10;
        g_ecu.engine_sync    = synced;
        g_ecu.air_mass_mg    = fo.air_mass_mg;
        g_ecu.ipw_us         = fo.ipw_final_us;
        g_ecu.ign_advance_deg= 10.0f;  // placeholder – ignition module below
        g_ecu.lambda_trim    = fo.lambda_trim;
        g_ecu.iac_duty       = iac_duty;

        seqlock_write_end();

        // ── 9. Maintain loop timing ───────────────────────────
        uint64_t elapsed = time_us_64() - tick_start;
        if (elapsed < period_us) {
            busy_wait_us(period_us - (uint32_t)elapsed);
        }
        // Overrun detection (logged on Core 1 via shared state)
    }

    return 0;   // unreachable
}
