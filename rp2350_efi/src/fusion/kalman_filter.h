/**
 * @file    kalman_filter.h
 * @brief   Linear Kalman Filter for EFI sensor fusion
 *
 * Two independent KF instances are used:
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ KF #1 – MAP Pressure + Rate Estimator                        │
 * │                                                              │
 * │  State vector  x = [ P_man  dP/dt ]ᵀ   (2-state)            │
 * │                P_man  = true manifold pressure [kPa]         │
 * │                dP/dt  = rate of change [kPa/s]               │
 * │                                                              │
 * │  Measurement   z = P_measured  (scalar, from ADC sensor)     │
 * │                                                              │
 * │  This gives us a smooth, derivative-aware pressure           │
 * │  estimate, which improves accel-enrichment prediction.       │
 * └──────────────────────────────────────────────────────────────┘
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ KF #2 – Intake Air Temperature Estimator (1-state)           │
 * │                                                              │
 * │  State vector  x = [ T_air ]                                 │
 * │  The NTC sensor has significant electrical noise;            │
 * │  a 1-state KF provides a low-latency, low-noise estimate.    │
 * └──────────────────────────────────────────────────────────────┘
 *
 * Air mass estimation
 * ───────────────────
 *  After KF updates, the air mass per intake event is:
 *
 *      ρ_air  = P_man_kPa × 1000 / (R_air × T_air_K)   [kg/m³]
 *      m_air  = ρ_air × (V_disp / N_cyl) × VE_pct/100   [kg]
 *
 *  where R_air = 287.05 J/(kg·K) and V_disp is cylinder displacement.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── MAP Kalman Filter (2-state) ───────────────────────────────

typedef struct {
    // State estimate: x[0] = P [kPa], x[1] = dP/dt [kPa/s]
    float x[2];
    // Error covariance matrix (2×2, stored row-major)
    float P[4];
    // Process noise covariance Q (diagonal, 2 elements)
    float Q[2];
    // Measurement noise variance R
    float R;
    // Last update timestamp (µs, for variable dt)
    uint64_t last_us;
    // Initialised flag
    bool     init;
} MapKalman_t;

/**
 * Initialise MAP KF with sensible defaults.
 * @param kf         pointer to MapKalman_t instance
 * @param initial_kpa  first pressure measurement to seed the state
 */
void map_kf_init(MapKalman_t *kf, float initial_kpa);

/**
 * Predict + update step.
 * @param kf          pointer to instance
 * @param measured_kpa  new MAP sensor reading [kPa]
 * @param now_us        current timestamp from time_us_64()
 */
void map_kf_update(MapKalman_t *kf, float measured_kpa, uint64_t now_us);

static inline float map_kf_get_pressure(const MapKalman_t *kf) { return kf->x[0]; }
static inline float map_kf_get_rate    (const MapKalman_t *kf) { return kf->x[1]; }


// ── IAT Kalman Filter (1-state) ───────────────────────────────

typedef struct {
    float    x;       // estimated temperature [°C]
    float    P;       // error variance
    float    Q;       // process noise variance
    float    R;       // measurement noise variance
    bool     init;
} IatKalman_t;

void  iat_kf_init  (IatKalman_t *kf, float initial_degC);
void  iat_kf_update(IatKalman_t *kf, float measured_degC);

static inline float iat_kf_get_temp(const IatKalman_t *kf) { return kf->x; }


// ── Air mass computation ──────────────────────────────────────

/**
 * Compute the estimated air mass per cylinder event [mg].
 *
 * @param map_kpa   KF-estimated MAP [kPa]
 * @param iat_degC  KF-estimated IAT [°C]
 * @param ve_pct    volumetric efficiency from table [%]
 * @returns         air mass per cylinder event [mg]
 */
float compute_air_mass_mg(float map_kpa, float iat_degC, float ve_pct);

#ifdef __cplusplus
}
#endif
