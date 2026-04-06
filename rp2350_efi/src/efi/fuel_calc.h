/**
 * @file    fuel_calc.h
 * @brief   EFI fuel calculation – IPW, enrichments, closed-loop trim
 *
 * Implements the Speed-Density fuel equation:
 *
 *   IPW = (m_air / AFR_target) / inj_capacity  +  dead_time
 *
 * where m_air is the KF-estimated air mass per cylinder event,
 * and inj_capacity is the injector flow rate converted to mg/µs.
 *
 * Enrichment cascade
 * ──────────────────
 *   IPW_base  = f(m_air, AFR_target)
 *   IPW_warm  = IPW_base  × warm_up_factor(CLT)
 *   IPW_accel = IPW_warm  + accel_enrichment(TPS_dot, RPM)
 *   IPW_final = IPW_accel × (1 + lambda_trim)   (closed-loop)
 *   IPW_out   = IPW_final + INJ_DEAD_TIME_US
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── VE table dimensions ───────────────────────────────────────
#define VE_RPM_BINS     16
#define VE_LOAD_BINS    16   // load = MAP [kPa] in Speed-Density

// VE table: rows = RPM axis, cols = MAP axis
// Values in percent (0-110).  Stored in ROM (flash) to save RAM.
extern const float VE_TABLE[VE_RPM_BINS][VE_LOAD_BINS];
extern const float VE_RPM_AXIS[VE_RPM_BINS];    // [RPM]
extern const float VE_LOAD_AXIS[VE_LOAD_BINS];  // [kPa]

// Target AFR table (rows = RPM, cols = MAP)
extern const float AFR_TABLE[VE_RPM_BINS][VE_LOAD_BINS];

// ── Fuel calc parameter structure ────────────────────────────

typedef struct {
    float rpm;           // [RPM]
    float map_kpa;       // KF-estimated MAP [kPa]
    float map_kpa_dot;   // KF rate dP/dt [kPa/s]  (for accel detection)
    float iat_degC;      // KF-estimated IAT [°C]
    float clt_degC;      // coolant temperature [°C]
    float tps_pct;       // throttle position [%]
    float tps_dot;       // d(TPS)/dt [%/s]         (for accel enrichment)
    float lambda_meas;   // measured λ from O2 sensor
    bool  closed_loop;   // true if CLT > 70°C and engine warm
} FuelInput_t;

typedef struct {
    float ve_pct;           // interpolated VE [%]
    float afr_target;       // interpolated target AFR
    float air_mass_mg;      // KF-derived air mass [mg]
    float ipw_base_us;      // base pulse width before enrichments [µs]
    float ipw_warmup_us;    // after cold-start enrichment [µs]
    float ipw_accel_us;     // accel enrichment added [µs]
    float lambda_trim;      // closed-loop correction [-0.25 … +0.25]
    float ipw_final_us;     // final IPW sent to injector [µs]
} FuelOutput_t;

/**
 * Initialise fuel calculation module.
 * Call once at startup.
 */
void fuel_calc_init(void);

/**
 * Run one fuel calculation cycle.
 * Must be called from the control loop at 2 kHz.
 * @param in   current sensor / state snapshot
 * @param out  populated with calculated fuel parameters
 */
void fuel_calc_run(const FuelInput_t *in, FuelOutput_t *out);

/**
 * Bilinear interpolation on a 2D table.
 * @param table   pointer to [RPM_BINS][LOAD_BINS] float array
 * @param rpm     current RPM
 * @param load    current load (MAP kPa)
 * @returns       interpolated value
 */
float table_lookup_2d(const float table[VE_RPM_BINS][VE_LOAD_BINS],
                      float rpm, float load);

#ifdef __cplusplus
}
#endif
