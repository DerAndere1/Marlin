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
 * Find parameter value in command string and parse colon-separated floats
 * @param cmd - Full command string (e.g., "M667 A1.0:2.0:3.0 B4.0:5.0")
 * @param param - Parameter letter to find (e.g., 'A')
 * @param arr - Output array
 * @param max_count - Maximum number of values to parse
 * @return Number of values parsed
 */
static int parse_param_floats(const char* cmd, char param, float* arr, int max_count) {
  // Find the parameter letter in the command
  const char* p = cmd;
  while (*p) {
    // Skip to next letter (parameters are uppercase letters)
    if (*p == param || *p == (param + 32)) {  // Match upper or lowercase
      p++;  // Move past the parameter letter
      break;
    }
    p++;
  }

  if (!*p) return 0;  // Parameter not found

  int count = 0;
  while (*p && count < max_count) {
    // Skip whitespace
    while (*p == ' ' || *p == '\t') p++;

    // Check if we hit another parameter (letter) or end
    if ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z')) break;
    if (!*p) break;

    // Parse float
    char* end;
    float val = strtof(p, &end);

    if (end == p) break;  // No valid float found

    arr[count++] = val;
    p = end;

    // Skip colon separator
    if (*p == ':') p++;
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

  // Get the raw command string for parsing colon-separated values
  const char* cmd = parser.command_ptr;

  // A - C-sweep X coefficients (7 values)
  if (parser.seen('A')) {
    int n = parse_param_floats(cmd, 'A', calibration_c_x, CALIBRATION_C_COEFFS);
    SERIAL_ECHOPGM("Set C-sweep X: ");
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" coefficients");
    report_coeffs("C-sweep X:", calibration_c_x, CALIBRATION_C_COEFFS);
  }

  // B - C-sweep Y coefficients
  if (parser.seen('B')) {
    int n = parse_param_floats(cmd, 'B', calibration_c_y, CALIBRATION_C_COEFFS);
    SERIAL_ECHOPGM("Set C-sweep Y: ");
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" coefficients");
    report_coeffs("C-sweep Y:", calibration_c_y, CALIBRATION_C_COEFFS);
  }

  // C - C-sweep Z coefficients
  if (parser.seen('C')) {
    int n = parse_param_floats(cmd, 'C', calibration_c_z, CALIBRATION_C_COEFFS);
    SERIAL_ECHOPGM("Set C-sweep Z: ");
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" coefficients");
    report_coeffs("C-sweep Z:", calibration_c_z, CALIBRATION_C_COEFFS);
  }

  // D - B-sweep X coefficients (5 values)
  if (parser.seen('D')) {
    int n = parse_param_floats(cmd, 'D', calibration_b_x, CALIBRATION_B_COEFFS);
    SERIAL_ECHOPGM("Set B-sweep X: ");
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" coefficients");
    report_coeffs("B-sweep X:", calibration_b_x, CALIBRATION_B_COEFFS);
  }

  // E - B-sweep Y coefficients
  if (parser.seen('E')) {
    int n = parse_param_floats(cmd, 'E', calibration_b_y, CALIBRATION_B_COEFFS);
    SERIAL_ECHOPGM("Set B-sweep Y: ");
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" coefficients");
    report_coeffs("B-sweep Y:", calibration_b_y, CALIBRATION_B_COEFFS);
  }

  // F - B-sweep Z coefficients
  if (parser.seen('F')) {
    int n = parse_param_floats(cmd, 'F', calibration_b_z, CALIBRATION_B_COEFFS);
    SERIAL_ECHOPGM("Set B-sweep Z: ");
    SERIAL_ECHO(n);
    SERIAL_ECHOLNPGM(" coefficients");
    report_coeffs("B-sweep Z:", calibration_b_z, CALIBRATION_B_COEFFS);
  }
}

#endif // CALIBRATION_CORRECTION
