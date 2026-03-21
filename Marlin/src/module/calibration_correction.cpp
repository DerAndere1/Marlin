/**
 * Rep5x Calibration Correction
 * Copyright 2025-2026 Dennis Klappe
 *
 * Applies Fourier-based calibration corrections to compensate for
 * mechanical errors in 5-axis kinematics.
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(CALIBRATION_CORRECTION)

#include "calibration_correction.h"
#include "../core/serial.h"

// Coefficient storage - initialized to zero (no correction)
float calibration_c_x[CALIBRATION_C_COEFFS] = {0};
float calibration_c_y[CALIBRATION_C_COEFFS] = {0};
float calibration_c_z[CALIBRATION_C_COEFFS] = {0};

float calibration_b_x[CALIBRATION_B_COEFFS] = {0};
float calibration_b_y[CALIBRATION_B_COEFFS] = {0};
float calibration_b_z[CALIBRATION_B_COEFFS] = {0};

bool calibration_correction_enabled = false;
bool calibration_for_move_target = false;

/**
 * Evaluate Fourier series: f(θ) = a0 + Σ(ak*cos(kθ) + bk*sin(kθ))
 */
float evaluate_fourier(const float* coeffs, int num_coeffs, float angle_deg) {
  const float theta = RADIANS(angle_deg);
  float result = coeffs[0];  // a0 (constant term)

  const int harmonics = (num_coeffs - 1) / 2;
  for (int k = 1; k <= harmonics; k++) {
    const float ak = coeffs[1 + 2 * (k - 1)];
    const float bk = coeffs[2 + 2 * (k - 1)];
    result += ak * cosf(k * theta) + bk * sinf(k * theta);
  }

  return result;
}

/**
 * Get calibration correction for given C/B angles
 * Uses additive model: total = c_correction + b_correction - baseline
 */
xyz_pos_t get_calibration_correction(float c, float b) {
  xyz_pos_t correction = { 0, 0, 0 };

  if (!calibration_correction_enabled) return correction;

  // Normalize C to 0-360 range
  c = fmodf(fmodf(c, 360.0f) + 360.0f, 360.0f);

  // Evaluate C sweep corrections
  const float cx = evaluate_fourier(calibration_c_x, CALIBRATION_C_COEFFS, c);
  const float cy = evaluate_fourier(calibration_c_y, CALIBRATION_C_COEFFS, c);
  const float cz = evaluate_fourier(calibration_c_z, CALIBRATION_C_COEFFS, c);

  // Evaluate B sweep corrections
  const float bx = evaluate_fourier(calibration_b_x, CALIBRATION_B_COEFFS, b);
  const float by = evaluate_fourier(calibration_b_y, CALIBRATION_B_COEFFS, b);
  const float bz = evaluate_fourier(calibration_b_z, CALIBRATION_B_COEFFS, b);

  // Baseline at C=0, B=0 (to avoid double-counting)
  const float base_x = evaluate_fourier(calibration_c_x, CALIBRATION_C_COEFFS, 0);
  const float base_y = evaluate_fourier(calibration_c_y, CALIBRATION_C_COEFFS, 0);
  const float base_z = evaluate_fourier(calibration_c_z, CALIBRATION_C_COEFFS, 0);

  // Additive model
  correction.x = cx + bx - base_x;
  correction.y = cy + by - base_y;
  correction.z = cz + bz - base_z;

  return correction;
}

/**
 * Reset all calibration coefficients to zero
 */
void reset_calibration_coefficients() {
  for (int i = 0; i < CALIBRATION_C_COEFFS; i++) {
    calibration_c_x[i] = 0;
    calibration_c_y[i] = 0;
    calibration_c_z[i] = 0;
  }
  for (int i = 0; i < CALIBRATION_B_COEFFS; i++) {
    calibration_b_x[i] = 0;
    calibration_b_y[i] = 0;
    calibration_b_z[i] = 0;
  }
  calibration_correction_enabled = false;
}

/**
 * Report calibration coefficients
 */
void report_calibration_coefficients() {
  SERIAL_ECHOLNPGM("Calibration Correction: ", calibration_correction_enabled ? "ON" : "OFF");

  SERIAL_ECHOPGM("C-sweep X:");
  for (int i = 0; i < CALIBRATION_C_COEFFS; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(calibration_c_x[i]);
  }
  SERIAL_EOL();

  SERIAL_ECHOPGM("C-sweep Y:");
  for (int i = 0; i < CALIBRATION_C_COEFFS; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(calibration_c_y[i]);
  }
  SERIAL_EOL();

  SERIAL_ECHOPGM("C-sweep Z:");
  for (int i = 0; i < CALIBRATION_C_COEFFS; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(calibration_c_z[i]);
  }
  SERIAL_EOL();

  SERIAL_ECHOPGM("B-sweep X:");
  for (int i = 0; i < CALIBRATION_B_COEFFS; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(calibration_b_x[i]);
  }
  SERIAL_EOL();

  SERIAL_ECHOPGM("B-sweep Y:");
  for (int i = 0; i < CALIBRATION_B_COEFFS; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(calibration_b_y[i]);
  }
  SERIAL_EOL();

  SERIAL_ECHOPGM("B-sweep Z:");
  for (int i = 0; i < CALIBRATION_B_COEFFS; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(calibration_b_z[i]);
  }
  SERIAL_EOL();
}

#endif // CALIBRATION_CORRECTION
