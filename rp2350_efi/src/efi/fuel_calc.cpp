/**
 * @file    fuel_calc.cpp
 * @brief   EFI fuel calculation – VE lookup, IPW, enrichments, O2 trim
 */

#include "fuel_calc.h"
#include "../ecu_config.h"
#include "../fusion/kalman_filter.h"

#include <math.h>
#include <string.h>

// ── Calibration tables (flash / ROM) ─────────────────────────
// These would normally be loaded from flash via TunerStudio protocol.
// Values here represent a generic 1600 cc 4-cylinder NA engine.

// RPM axis:  800 … 7000 RPM (16 bins)
const float VE_RPM_AXIS[VE_RPM_BINS] = {
     800, 1000, 1200, 1500, 1800, 2200, 2600, 3000,
    3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000
};

// Load axis: MAP 20 … 100 kPa (16 bins, 5 kPa steps)
const float VE_LOAD_AXIS[VE_LOAD_BINS] = {
    20, 25, 30, 35, 40, 50, 60, 70,
    75, 80, 85, 88, 90, 92, 96, 100
};

// VE table [RPM_BINS][LOAD_BINS]  –  values in percent
// Rows = RPM (low→high), Cols = MAP kPa (low→high)
const float VE_TABLE[VE_RPM_BINS][VE_LOAD_BINS] = {
    /* 800  */ { 58, 60, 62, 64, 66, 68, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80 },
    /* 1000 */ { 60, 62, 64, 66, 68, 70, 72, 74, 75, 76, 77, 78, 79, 80, 81, 82 },
    /* 1200 */ { 62, 64, 66, 68, 70, 72, 74, 76, 77, 78, 79, 80, 81, 82, 83, 84 },
    /* 1500 */ { 64, 66, 68, 70, 72, 74, 76, 78, 79, 80, 81, 82, 83, 84, 85, 86 },
    /* 1800 */ { 65, 67, 69, 71, 73, 75, 77, 79, 80, 81, 82, 83, 84, 85, 86, 87 },
    /* 2200 */ { 66, 68, 70, 72, 74, 76, 78, 80, 81, 82, 83, 84, 85, 86, 87, 88 },
    /* 2600 */ { 67, 69, 71, 73, 75, 77, 79, 81, 82, 83, 84, 85, 86, 87, 88, 89 },
    /* 3000 */ { 68, 70, 72, 74, 76, 78, 80, 82, 83, 84, 85, 86, 87, 88, 89, 90 },
    /* 3500 */ { 69, 71, 73, 75, 77, 79, 81, 83, 84, 85, 86, 87, 88, 89, 90, 91 },
    /* 4000 */ { 70, 72, 74, 76, 78, 80, 82, 84, 85, 86, 87, 88, 89, 90, 91, 92 },
    /* 4500 */ { 69, 71, 73, 75, 77, 79, 81, 83, 84, 85, 86, 87, 88, 89, 90, 91 },
    /* 5000 */ { 68, 70, 72, 74, 76, 78, 80, 82, 83, 84, 85, 86, 87, 88, 89, 90 },
    /* 5500 */ { 66, 68, 70, 72, 74, 76, 78, 80, 81, 82, 83, 84, 85, 86, 87, 88 },
    /* 6000 */ { 64, 66, 68, 70, 72, 74, 76, 78, 79, 80, 81, 82, 83, 84, 85, 86 },
    /* 6500 */ { 62, 64, 66, 68, 70, 72, 74, 76, 77, 78, 79, 80, 81, 82, 83, 84 },
    /* 7000 */ { 60, 62, 64, 66, 68, 70, 72, 74, 75, 76, 77, 78, 79, 80, 81, 82 },
};

// Target AFR table [RPM_BINS][LOAD_BINS]
// High load → richer (lower AFR); cruise → stoich or lean
const float AFR_TABLE[VE_RPM_BINS][VE_LOAD_BINS] = {
    /* 800  */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 1000 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 1200 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 1500 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 1800 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 2200 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 2600 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f },
    /* 3000 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f },
    /* 3500 */ { 14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,14.0f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f },
    /* 4000 */ { 14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f },
    /* 4500 */ { 14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f },
    /* 5000 */ { 14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f },
    /* 5500 */ { 14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f },
    /* 6000 */ { 14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f,11.6f },
    /* 6500 */ { 14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f,11.6f,11.5f },
    /* 7000 */ { 13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f,11.6f,11.5f,11.4f },
};

// ── Injector capacity in mg/µs ────────────────────────────────
// INJ_FLOW_RATE_CC_MIN @ 3.0 bar → mg/µs
// Fuel density ≈ 0.74 g/cc (petrol)
// mg/µs = (cc/min × 740 mg/cc) / (60 s × 1e6 µs/s)
static float s_inj_mg_per_us;

// ── Closed-loop integrator state ──────────────────────────────
static float s_lambda_integrator = 0.0f;
static constexpr float LAMBDA_KI  = 0.001f;  // integrator gain per control tick
static constexpr float LAMBDA_MAX = 0.25f;   // ±25% trim limit

// ── Accel enrichment state ────────────────────────────────────
static float s_accel_tps_prev  = 0.0f;
static float s_accel_decay_us  = 0.0f;  // remaining accel pulse width [µs]
static constexpr float ACCEL_DECAY_RATE = 0.85f;  // per control tick (exponential)
static constexpr float ACCEL_GAIN_US_PCT_PER_S = 0.5f; // µs per (%/s) of TPS rate


// ── 2D bilinear table interpolation ──────────────────────────

float table_lookup_2d(const float table[VE_RPM_BINS][VE_LOAD_BINS],
                      float rpm, float load) {
    // Find bounding RPM bins
    int ri = 0;
    for (int i = 0; i < VE_RPM_BINS - 1; i++) {
        if (rpm >= VE_RPM_AXIS[i]) ri = i;
    }

    // Find bounding load bins
    int li = 0;
    for (int i = 0; i < VE_LOAD_BINS - 1; i++) {
        if (load >= VE_LOAD_AXIS[i]) li = i;
    }

    // Clamp indices
    if (ri >= VE_RPM_BINS - 1) ri = VE_RPM_BINS - 2;
    if (li >= VE_LOAD_BINS - 1) li = VE_LOAD_BINS - 2;

    // Fractional position within cell
    float rpm_frac  = (rpm  - VE_RPM_AXIS[ri])  / (VE_RPM_AXIS[ri+1]  - VE_RPM_AXIS[ri]);
    float load_frac = (load - VE_LOAD_AXIS[li]) / (VE_LOAD_AXIS[li+1] - VE_LOAD_AXIS[li]);

    // Bilinear interpolation
    float v00 = table[ri  ][li  ];
    float v10 = table[ri+1][li  ];
    float v01 = table[ri  ][li+1];
    float v11 = table[ri+1][li+1];

    return v00 * (1-rpm_frac)*(1-load_frac)
         + v10 * rpm_frac    *(1-load_frac)
         + v01 * (1-rpm_frac)* load_frac
         + v11 * rpm_frac    * load_frac;
}

// ── Cold start warm-up multiplier ─────────────────────────────
// Returns extra fuel factor based on CLT.
// At -40°C → factor = 2.0 (double fuel); at 80°C → factor = 1.0 (no extra)
static float warmup_factor(float clt_degC) {
    if (clt_degC >= 80.0f) return 1.0f;
    if (clt_degC <= -40.0f) return 2.0f;
    // Linear interpolation
    return 1.0f + (80.0f - clt_degC) / 120.0f;
}

// ── Init ─────────────────────────────────────────────────────

void fuel_calc_init(void) {
    // Convert injector flow rate to mg/µs
    // cc/min × 0.74 g/cc × 1000 mg/g / (60 s/min × 1e6 µs/s)
    s_inj_mg_per_us = (INJ_FLOW_RATE_CC_MIN * 740.0f) / (60.0f * 1.0e6f);
    s_lambda_integrator = 0.0f;
    s_accel_decay_us    = 0.0f;
    s_accel_tps_prev    = 0.0f;
}

// ── Main fuel calculation ─────────────────────────────────────

void fuel_calc_run(const FuelInput_t *in, FuelOutput_t *out) {
    // ── 1. VE and AFR lookup ──────────────────────────────────
    out->ve_pct     = table_lookup_2d(VE_TABLE,  in->rpm, in->map_kpa);
    out->afr_target = table_lookup_2d(AFR_TABLE, in->rpm, in->map_kpa);

    // ── 2. Air mass from Ideal Gas Law (KF-estimated inputs) ──
    out->air_mass_mg = compute_air_mass_mg(in->map_kpa, in->iat_degC, out->ve_pct);

    // ── 3. Base IPW ───────────────────────────────────────────
    // Fuel mass required = air_mass / AFR_target  [mg]
    float fuel_mass_mg  = out->air_mass_mg / out->afr_target;
    // Pulse width = fuel_mass / injector_capacity  [µs]
    out->ipw_base_us    = fuel_mass_mg / s_inj_mg_per_us;

    // ── 4. Cold-start warm-up enrichment ─────────────────────
    float wf = warmup_factor(in->clt_degC);
    out->ipw_warmup_us = out->ipw_base_us * wf;

    // ── 5. Acceleration enrichment ───────────────────────────
    // Detect rapid TPS increase → add accel fuel pulse
    // tps_dot > 50 %/s → trigger enrichment
    if (in->tps_dot > 50.0f) {
        float new_accel = in->tps_dot * ACCEL_GAIN_US_PCT_PER_S;
        if (new_accel > s_accel_decay_us) {
            s_accel_decay_us = new_accel;  // latch the larger value
        }
    }
    out->ipw_accel_us  = out->ipw_warmup_us + s_accel_decay_us;
    // Decay accel enrichment each tick
    s_accel_decay_us  *= ACCEL_DECAY_RATE;
    if (s_accel_decay_us < 0.5f) s_accel_decay_us = 0.0f;

    // ── 6. Closed-loop lambda trim ────────────────────────────
    if (in->closed_loop && in->lambda_meas > 0.5f && in->lambda_meas < 2.0f) {
        // Error: want λ = 1.0  (target lambda = AFR_target / STOICH_AFR)
        float lambda_target = out->afr_target / STOICH_AFR;
        float lambda_error  = lambda_target - in->lambda_meas;
        // Integrate error (I-term only for simplicity; P-term can be added)
        s_lambda_integrator += lambda_error * LAMBDA_KI;
        // Clamp integrator
        if (s_lambda_integrator >  LAMBDA_MAX) s_lambda_integrator =  LAMBDA_MAX;
        if (s_lambda_integrator < -LAMBDA_MAX) s_lambda_integrator = -LAMBDA_MAX;
    }
    out->lambda_trim = s_lambda_integrator;

    // ── 7. Apply trim ─────────────────────────────────────────
    float ipw_trimmed = out->ipw_accel_us * (1.0f + out->lambda_trim);

    // ── 8. Add injector dead time ─────────────────────────────
    out->ipw_final_us = ipw_trimmed + INJ_DEAD_TIME_US;

    // ── 9. Sanity limits ──────────────────────────────────────
    if (out->ipw_final_us <    0.0f) out->ipw_final_us =    0.0f;
    if (out->ipw_final_us > 20000.0f) out->ipw_final_us = 20000.0f;  // 20 ms max
}
