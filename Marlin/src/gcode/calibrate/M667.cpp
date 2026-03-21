/**
 * Rep5x Calibration Correction G-code
 * Copyright 2025-2026 Dennis Klappe
 *
 * M667: Set/Get calibration correction coefficients
 *
 * Usage:
 *   M667           - Report current coefficients and status
 *   M667 S0        - Disable calibration correction
 *   M667 S1        - Enable calibration correction
 *   M667 R         - Reset all coefficients to zero
 *
 * Set C-sweep coefficients (7 values, colon-separated):
 *   M667 A<c0>:<c1>:<c2>:<c3>:<c4>:<c5>:<c6>  - Set C-sweep X coefficients
 *   M667 B<c0>:<c1>:<c2>:<c3>:<c4>:<c5>:<c6>  - Set C-sweep Y coefficients
 *   M667 C<c0>:<c1>:<c2>:<c3>:<c4>:<c5>:<c6>  - Set C-sweep Z coefficients
 *
 * Set B-sweep coefficients (5 values, colon-separated):
 *   M667 D<c0>:<c1>:<c2>:<c3>:<c4>  - Set B-sweep X coefficients
 *   M667 E<c0>:<c1>:<c2>:<c3>:<c4>  - Set B-sweep Y coefficients
 *   M667 F<c0>:<c1>:<c2>:<c3>:<c4>  - Set B-sweep Z coefficients
 *
 * Coefficients are Fourier series: f(θ) = a0 + a1*cos(θ) + b1*sin(θ) + ...
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(CALIBRATION_CORRECTION)

#include "../gcode.h"
#include "../../module/calibration_correction.h"

/**
 * Parse colon-separated float values into an array
 * @param str - String like "1.23:-4.56:7.89"
 * @param arr - Output array
 * @param max_count - Maximum number of values to parse
 * @return Number of values parsed
 */
static int parse_colon_floats(const char* str, float* arr, int max_count) {
  int count = 0;
  const char* p = str;

  while (*p && count < max_count) {
    // Skip leading whitespace
    while (*p == ' ' || *p == '\t') p++;

    // Parse float
    char* end;
    float val = strtof(p, &end);

    if (end == p) break;  // No valid float found

    arr[count++] = val;
    p = end;

    // Skip colon separator
    if (*p == ':') p++;
    else if (*p && *p != ' ' && *p != '\t') break;  // Invalid character
  }

  return count;
}

/**
 * Report array of coefficients
 */
static void report_coeffs(const char* name, const float* arr, int count) {
  SERIAL_ECHOPGM("  ");
  SERIAL_ECHOPGM(name);
  for (int i = 0; i < count; i++) {
    SERIAL_CHAR(' ');
    SERIAL_ECHO(arr[i]);
  }
  SERIAL_EOL();
}

void GcodeSuite::M667() {
  // No parameters - report current state
  if (!parser.seen_any()) {
    report_calibration_coefficients();
    return;
  }

  // S - Enable/disable
  if (parser.seen('S')) {
    calibration_correction_enabled = parser.value_bool();
    SERIAL_ECHOLNPGM("Calibration correction: ", calibration_correction_enabled ? "ON" : "OFF");
  }

  // R - Reset coefficients
  if (parser.seen('R')) {
    reset_calibration_coefficients();
    SERIAL_ECHOLNPGM("Calibration coefficients reset");
  }

  // A - C-sweep X coefficients (7 values)
  if (parser.seen('A')) {
    const char* val = parser.string_arg;
    if (val) {
      int n = parse_colon_floats(val, calibration_c_x, CALIBRATION_C_COEFFS);
      SERIAL_ECHOPGM("Set C-sweep X: ");
      SERIAL_ECHO(n);
      SERIAL_ECHOLNPGM(" coefficients");
      report_coeffs("C-sweep X:", calibration_c_x, CALIBRATION_C_COEFFS);
    }
  }

  // B - C-sweep Y coefficients
  if (parser.seen('B')) {
    const char* val = parser.string_arg;
    if (val) {
      int n = parse_colon_floats(val, calibration_c_y, CALIBRATION_C_COEFFS);
      SERIAL_ECHOPGM("Set C-sweep Y: ");
      SERIAL_ECHO(n);
      SERIAL_ECHOLNPGM(" coefficients");
      report_coeffs("C-sweep Y:", calibration_c_y, CALIBRATION_C_COEFFS);
    }
  }

  // C - C-sweep Z coefficients
  if (parser.seen('C')) {
    const char* val = parser.string_arg;
    if (val) {
      int n = parse_colon_floats(val, calibration_c_z, CALIBRATION_C_COEFFS);
      SERIAL_ECHOPGM("Set C-sweep Z: ");
      SERIAL_ECHO(n);
      SERIAL_ECHOLNPGM(" coefficients");
      report_coeffs("C-sweep Z:", calibration_c_z, CALIBRATION_C_COEFFS);
    }
  }

  // D - B-sweep X coefficients (5 values)
  if (parser.seen('D')) {
    const char* val = parser.string_arg;
    if (val) {
      int n = parse_colon_floats(val, calibration_b_x, CALIBRATION_B_COEFFS);
      SERIAL_ECHOPGM("Set B-sweep X: ");
      SERIAL_ECHO(n);
      SERIAL_ECHOLNPGM(" coefficients");
      report_coeffs("B-sweep X:", calibration_b_x, CALIBRATION_B_COEFFS);
    }
  }

  // E - B-sweep Y coefficients
  if (parser.seen('E')) {
    const char* val = parser.string_arg;
    if (val) {
      int n = parse_colon_floats(val, calibration_b_y, CALIBRATION_B_COEFFS);
      SERIAL_ECHOPGM("Set B-sweep Y: ");
      SERIAL_ECHO(n);
      SERIAL_ECHOLNPGM(" coefficients");
      report_coeffs("B-sweep Y:", calibration_b_y, CALIBRATION_B_COEFFS);
    }
  }

  // F - B-sweep Z coefficients
  if (parser.seen('F')) {
    const char* val = parser.string_arg;
    if (val) {
      int n = parse_colon_floats(val, calibration_b_z, CALIBRATION_B_COEFFS);
      SERIAL_ECHOPGM("Set B-sweep Z: ");
      SERIAL_ECHO(n);
      SERIAL_ECHOLNPGM(" coefficients");
      report_coeffs("B-sweep Z:", calibration_b_z, CALIBRATION_B_COEFFS);
    }
  }
}

#endif // CALIBRATION_CORRECTION
