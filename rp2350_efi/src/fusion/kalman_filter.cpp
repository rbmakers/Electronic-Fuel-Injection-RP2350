/**
 * @file    kalman_filter.cpp
 * @brief   Linear Kalman Filter implementations for MAP and IAT
 *
 * MAP KF – 2-state constant-velocity model
 * ─────────────────────────────────────────
 *  System model (discrete, variable dt):
 *
 *    F = | 1  dt |      transition matrix
 *        | 0   1 |
 *
 *    H = [ 1  0 ]        measurement matrix (we observe pressure only)
 *
 *  Predict:
 *    x_pred = F × x
 *    P_pred = F × P × Fᵀ + Q
 *
 *  Update (scalar measurement):
 *    y  = z - H × x_pred          (innovation)
 *    S  = H × P_pred × Hᵀ + R    (innovation covariance, scalar here)
 *    K  = P_pred × Hᵀ / S        (Kalman gain, 2×1 vector)
 *    x  = x_pred + K × y
 *    P  = (I - K×H) × P_pred
 *
 * The MAP noise parameters are tuned for a typical 0-250 kPa sensor
 * with ~1-2 kPa RMS electrical noise and engine dynamics up to ~200 kPa/s.
 */

#include "kalman_filter.h"
#include "../ecu_config.h"

#include "hardware/timer.h"
#include <math.h>
#include <string.h>

// ── Physical constants ────────────────────────────────────────
static constexpr float R_AIR_J_KGK = 287.05f;  // specific gas constant for dry air

// ── MAP KF (2-state) ─────────────────────────────────────────

void map_kf_init(MapKalman_t *kf, float initial_kpa) {
    kf->x[0] = initial_kpa;
    kf->x[1] = 0.0f;        // assume dP/dt = 0 at start

    // P = large initial covariance (uncertain initial state)
    kf->P[0] = 100.0f;  // var(P)
    kf->P[1] = 0.0f;
    kf->P[2] = 0.0f;
    kf->P[3] = 50.0f;   // var(dP/dt)

    // Process noise: pressure changes ~0.5 kPa/s²; rate changes ~5 kPa/s²/s
    kf->Q[0] = 0.1f;    // Q_pp   [kPa²]
    kf->Q[1] = 1.0f;    // Q_dd   [(kPa/s)²]

    // Measurement noise: MAP sensor RMS noise ~1.5 kPa
    kf->R = 2.25f;      // 1.5² [kPa²]

    kf->last_us = time_us_64();
    kf->init    = true;
}

void map_kf_update(MapKalman_t *kf, float measured_kpa, uint64_t now_us) {
    if (!kf->init) {
        map_kf_init(kf, measured_kpa);
        return;
    }

    // ── Time delta ─────────────────────────────────────────────
    float dt = (float)(now_us - kf->last_us) * 1.0e-6f;  // µs → seconds
    kf->last_us = now_us;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.0005f;   // clamp: 0 to 100 ms

    // ── Predict ────────────────────────────────────────────────
    // State prediction: x_pred = F × x
    //   F = [[1, dt], [0, 1]]
    float xp0 = kf->x[0] + kf->x[1] * dt;   // predicted pressure
    float xp1 = kf->x[1];                    // predicted rate (constant)

    // Covariance prediction: P_pred = F × P × Fᵀ + Q
    // With F = [[1,dt],[0,1]] and diagonal Q:
    //   P_pred[0,0] = P00 + dt*(P10+P01) + dt²*P11 + Q0
    //   P_pred[0,1] = P_pred[1,0] = P01 + dt*P11
    //   P_pred[1,1] = P11 + Q1
    float p00 = kf->P[0];
    float p01 = kf->P[1];
    float p10 = kf->P[2];
    float p11 = kf->P[3];

    float pp00 = p00 + dt * (p10 + p01) + dt * dt * p11 + kf->Q[0];
    float pp01 = p01 + dt * p11;
    float pp10 = p10 + dt * p11;
    float pp11 = p11 + kf->Q[1];

    // ── Update ─────────────────────────────────────────────────
    // H = [1, 0]  →  H×x_pred = xp0
    float y = measured_kpa - xp0;           // innovation

    // S = H×P_pred×Hᵀ + R = pp00 + R  (scalar)
    float S = pp00 + kf->R;

    // Kalman gain K = P_pred×Hᵀ / S  (2×1 vector)
    //   K = [pp00/S, pp10/S]
    float K0 = pp00 / S;
    float K1 = pp10 / S;

    // State update
    kf->x[0] = xp0 + K0 * y;
    kf->x[1] = xp1 + K1 * y;

    // Covariance update: P = (I - K×H) × P_pred
    // I - K×H = [[1-K0, 0], [-K1, 1]]
    kf->P[0] = (1.0f - K0) * pp00;
    kf->P[1] = (1.0f - K0) * pp01;
    kf->P[2] = -K1 * pp00 + pp10;
    kf->P[3] = -K1 * pp01 + pp11;

    // Clamp pressure estimate to physical limits
    if (kf->x[0] <   0.0f) kf->x[0] =   0.0f;
    if (kf->x[0] > 250.0f) kf->x[0] = 250.0f;
}


// ── IAT KF (1-state) ─────────────────────────────────────────

void iat_kf_init(IatKalman_t *kf, float initial_degC) {
    kf->x    = initial_degC;
    kf->P    = 100.0f;   // uncertain initially
    kf->Q    = 0.01f;    // temperature changes very slowly in steady-state
    kf->R    = 4.0f;     // NTC RMS noise ~2°C → R = 4
    kf->init = true;
}

void iat_kf_update(IatKalman_t *kf, float measured_degC) {
    if (!kf->init) { iat_kf_init(kf, measured_degC); return; }

    // Predict (constant model: x_pred = x)
    float xp = kf->x;
    float Pp = kf->P + kf->Q;

    // Update
    float S = Pp + kf->R;
    float K = Pp / S;
    kf->x = xp + K * (measured_degC - xp);
    kf->P = (1.0f - K) * Pp;

    // Physical clamp
    if (kf->x < -40.0f) kf->x = -40.0f;
    if (kf->x > 120.0f) kf->x = 120.0f;
}


// ── Air mass computation ──────────────────────────────────────

float compute_air_mass_mg(float map_kpa, float iat_degC, float ve_pct) {
    // Convert units
    float P_pa  = map_kpa * 1000.0f;           // kPa → Pa
    float T_K   = iat_degC + 273.15f;           // °C  → K

    // Ideal gas: ρ = P / (R × T)  [kg/m³]
    float rho_air = P_pa / (R_AIR_J_KGK * T_K);

    // Cylinder swept volume [m³]
    // V_swept = total_displacement / num_cylinders  (cc → m³)
    float V_swept_m3 = (ENGINE_DISPLACEMENT_CC / ENGINE_CYL) * 1.0e-6f;

    // Actual air mass per cylinder per event [kg]
    float m_air_kg = rho_air * V_swept_m3 * (ve_pct / 100.0f);

    // Convert to milligrams
    return m_air_kg * 1.0e6f;
}
