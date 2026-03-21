/**
 * Rep5x Calibration Correction
 * Copyright 2025-2026 Dennis Klappe
 *
 * Applies Fourier-based calibration corrections to compensate for
 * mechanical errors in 5-axis kinematics.
 *
 * Uses coefficients from the Rep5x Calibrator tool.
 */

#pragma once

#include "../core/types.h"

// Number of harmonics for Fourier fitting
#define CALIBRATION_C_HARMONICS 3  // 7 coefficients per axis
#define CALIBRATION_B_HARMONICS 2  // 5 coefficients per axis

// Total coefficients: C sweep = 7*3 = 21, B sweep = 5*3 = 15, Total = 36
#define CALIBRATION_C_COEFFS (1 + 2 * CALIBRATION_C_HARMONICS)  // 7
#define CALIBRATION_B_COEFFS (1 + 2 * CALIBRATION_B_HARMONICS)  // 5

// Coefficient storage
// C sweep: periodic 0-360 degrees, format [a0, a1, b1, a2, b2, a3, b3]
extern float calibration_c_x[CALIBRATION_C_COEFFS];
extern float calibration_c_y[CALIBRATION_C_COEFFS];
extern float calibration_c_z[CALIBRATION_C_COEFFS];

// B sweep: -90 to 90 degrees, format [c0, c1, s1, c2, s2]
extern float calibration_b_x[CALIBRATION_B_COEFFS];
extern float calibration_b_y[CALIBRATION_B_COEFFS];
extern float calibration_b_z[CALIBRATION_B_COEFFS];

// Enable/disable calibration correction
extern bool calibration_correction_enabled;

// Flag to indicate we're calculating a move target (not position sync)
// Calibration is only applied when this is true
extern bool calibration_for_move_target;

/**
 * Get calibration correction for given C/B angles
 * @param c C-axis angle in degrees
 * @param b B-axis angle in degrees
 * @return XYZ correction to ADD to machine coordinates
 */
xyz_pos_t get_calibration_correction(float c, float b);

/**
 * Evaluate Fourier series at given angle
 * @param coeffs Coefficient array [a0, a1, b1, a2, b2, ...]
 * @param num_coeffs Number of coefficients
 * @param angle_deg Angle in degrees
 * @return Evaluated value
 */
float evaluate_fourier(const float* coeffs, int num_coeffs, float angle_deg);

/**
 * Reset all calibration coefficients to zero
 */
void reset_calibration_coefficients();

/**
 * Report calibration coefficients (for M667 without parameters)
 */
void report_calibration_coefficients();
