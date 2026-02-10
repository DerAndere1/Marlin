/**
 * Rep5x IK Pre-processor
 * Copyright 2025-2026 Dennis Klappe
 *
 * Pre-processes G-code through inverse kinematics before printing.
 * Transforms tool-space coordinates to joint-space so the printer
 * can execute with TCPC disabled (G49), avoiding real-time IK jitter.
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(IK_PREPROCESS)

#include "ik_preprocessor.h"
#include "cardreader.h"
#include "../module/motion.h"
#include "../module/penta_axis_head_head.h"
#include "../module/temperature.h"
#include "../lcd/marlinui.h"

#if ENABLED(CALIBRATION_CORRECTION)
  #include "../module/calibration_correction.h"
#endif

#define IK_LINE_MAXLEN 128
#define IK_TEMP_FILENAME "_iktmp.gco"
#define IK_SUBDIVISION_DEG 1.0f

// G-code axis letters for the rotational axes
// Input files may use either the axis name (C/B) or positional letter (I/J)
#if AXIS4_NAME == 'C'
  #define IK_AXIS4_LETTER 'C'
  #define IK_AXIS5_LETTER 'B'
#else
  #define IK_AXIS4_LETTER 'I'
  #define IK_AXIS5_LETTER 'J'
#endif

// Static buffers to avoid stack overflow on STM32
static char line_buf[IK_LINE_MAXLEN];
static char out_buf[IK_LINE_MAXLEN];
static MediaFile tempfile;

// Read one line from the currently open SD card file.
// Returns false at EOF.
static bool read_line(char *buf, uint16_t maxlen) {
  uint16_t i = 0;
  while (i < maxlen - 1) {
    const int16_t c = card.get();
    if (c < 0) {  // EOF
      buf[i] = '\0';
      return i > 0;  // Return true if we got any chars before EOF
    }
    if (c == '\n') break;
    if (c == '\r') continue;  // Skip carriage returns
    buf[i++] = (char)c;
  }
  buf[i] = '\0';
  return true;
}

// Parse a float value for a given parameter letter from a G-code line.
// Param should be uppercase. Returns true if found, stores value in &out.
static bool parse_gcode_value(const char *line, const char param, float &out) {
  for (const char *p = line; *p; p++) {
    if ((*p == param || *p == (param | 0x20)) && p[1]) {
      char *end;
      const float val = strtof(p + 1, &end);
      if (end != p + 1) {
        out = val;
        return true;
      }
    }
  }
  return false;
}

// Parse a rotational axis value, trying both the axis name (C/B) and positional letter (I/J)
static bool parse_axis4(const char *line, float &out) {
  return parse_gcode_value(line, IK_AXIS4_LETTER, out) || parse_gcode_value(line, 'I', out);
}
static bool parse_axis5(const char *line, float &out) {
  return parse_gcode_value(line, IK_AXIS5_LETTER, out) || parse_gcode_value(line, 'J', out);
}

// Write a string to the temp file. Returns false on write error.
static bool write_line(const char *line) {
  const uint16_t len = strlen(line);
  if (len > 0 && tempfile.write(line, len) != (int16_t)len) return false;
  if (tempfile.write("\n", 1) != 1) return false;
  return true;
}

// Format and write a transformed G1 line using static out_buf
static bool write_transformed_line(
  const bool is_g0,
  const abce_pos_t &transformed,
  const float i_val, const float j_val,
  const bool has_e, const float e_val,
  const bool has_f, const float f_val
) {
  int pos = sprintf(out_buf, "%s X%.3f Y%.3f Z%.3f I%.3f J%.3f",
    is_g0 ? "G0" : "G1",
    (double)transformed.x, (double)transformed.y, (double)transformed.z,
    (double)i_val, (double)j_val);
  if (has_e) pos += sprintf(out_buf + pos, " E%.5f", (double)e_val);
  if (has_f) pos += sprintf(out_buf + pos, " F%.0f", (double)f_val);
  return write_line(out_buf);
}

// Lightweight keepalive: feed watchdog + manage heaters without full idle() overhead
static void keepalive() {
  thermalManager.task();  // Manages heaters and feeds watchdog
}

bool preprocess_ik_file() {
  if (!card.isMounted() || !card.isFileOpen()) return false;

  const uint32_t source_size = card.getFileSize();

  // Open temp file for writing
  if (!tempfile.open(&card.getWorkDir(), IK_TEMP_FILENAME, O_CREAT | O_WRITE | O_TRUNC)) {
    SERIAL_ERROR_MSG("M668: Failed to create temp file");
    return false;
  }

  ui.set_status(F("Processing IK..."));
  SERIAL_ECHOLNPGM("M668: IK pre-processing started");

  // Write G49 as first line to disable TCPC in the processed file
  if (!write_line("G49")) {
    SERIAL_ERROR_MSG("M668: Write error");
    tempfile.close();
    card.removeFile(IK_TEMP_FILENAME);
    return false;
  }

  // Save and set IK flags
  const bool saved_tcpc = tool_centerpoint_control;
  tool_centerpoint_control = true;
  #if ENABLED(CALIBRATION_CORRECTION)
    const bool saved_cal = calibration_for_move_target;
    calibration_for_move_target = true;
  #endif

  // Tracking state
  float track_x = 0, track_y = 0, track_z = 0;  // Current X, Y, Z
  float track_i = 0, track_j = 0;               // Current C, B angles
  bool absolute_mode = true;
  bool success = true;
  uint32_t line_count = 0;

  while (read_line(line_buf, IK_LINE_MAXLEN)) {
    line_count++;

    // Feed watchdog + manage heaters (lightweight, no UI/button processing)
    keepalive();

    // Progress update every 500 lines
    if ((line_count % 500) == 0) {
      const uint8_t pct = (uint8_t)((uint32_t)card.getIndex() * 100 / source_size);
      SERIAL_ECHOLNPGM("M668: ", pct, "% processed");
    }

    // Skip empty lines and comments — copy as-is
    if (line_buf[0] == '\0' || line_buf[0] == ';') {
      if (!write_line(line_buf)) { success = false; break; }
      continue;
    }

    // Detect command type
    const bool is_g = (line_buf[0] == 'G' || line_buf[0] == 'g');
    int cmd_num = -1;
    if (is_g) cmd_num = atoi(line_buf + 1);

    // G43.4 — skip (replaced by G49 at top)
    if (is_g && strstr(line_buf, "G43.4")) continue;
    // G49 — skip (already written at top)
    if (is_g && cmd_num == 49) continue;

    // G0 / G1 — transform through IK
    if (is_g && (cmd_num == 0 || cmd_num == 1)) {
      const bool is_g0 = (cmd_num == 0);

      // Parse parameters (supports both C/B and I/J axis letters)
      float new_x = track_x, new_y = track_y, new_z = track_z;
      float new_i = track_i, new_j = track_j;
      float e_val = 0;
      float f_val = 0;
      bool has_x = parse_gcode_value(line_buf, 'X', new_x);
      bool has_y = parse_gcode_value(line_buf, 'Y', new_y);
      bool has_z = parse_gcode_value(line_buf, 'Z', new_z);
      bool has_i = parse_axis4(line_buf, new_i);
      bool has_j = parse_axis5(line_buf, new_j);
      bool has_e = parse_gcode_value(line_buf, 'E', e_val);
      bool has_f = parse_gcode_value(line_buf, 'F', f_val);

      // Handle relative mode offsets
      if (!absolute_mode) {
        if (has_x) new_x += track_x;
        if (has_y) new_y += track_y;
        if (has_z) new_z += track_z;
        if (has_i) new_i += track_i;
        if (has_j) new_j += track_j;
      }

      // Calculate angle deltas for subdivision
      const float delta_i = new_i - track_i;
      const float delta_j = new_j - track_j;
      const float max_angle_delta = _MAX(ABS(delta_i), ABS(delta_j));
      const int segments = (max_angle_delta > IK_SUBDIVISION_DEG)
                           ? (int)ceilf(max_angle_delta / IK_SUBDIVISION_DEG)
                           : 1;

      // Starting position for interpolation
      const float start_x = track_x, start_y = track_y, start_z = track_z;
      const float start_i = track_i, start_j = track_j;

      for (int seg = 1; seg <= segments; seg++) {
        const float t = (float)seg / (float)segments;

        // Interpolate position
        const float seg_x = start_x + (new_x - start_x) * t;
        const float seg_y = start_y + (new_y - start_y) * t;
        const float seg_z = start_z + (new_z - start_z) * t;
        const float seg_i = start_i + delta_i * t;
        const float seg_j = start_j + delta_j * t;
        const float seg_e = has_e ? e_val * t : 0;

        // Build position for IK
        xyz_pos_t ik_input;
        ik_input.x = seg_x;
        ik_input.y = seg_y;
        ik_input.z = seg_z;
        #if AXIS4_NAME == 'C'
          ik_input.i = seg_i;  // C angle
          ik_input.j = seg_j;  // B angle
        #elif AXIS5_NAME == 'C'
          ik_input.j = seg_i;  // C angle
          ik_input.i = seg_j;  // B angle
        #endif

        // Run inverse kinematics (result stored in global `delta`)
        inverse_kinematics(ik_input);
        const bool seg_has_f = has_f && (seg == 1);  // F only on first segment
        if (!write_transformed_line(is_g0, delta, seg_i, seg_j,
                                    has_e, seg_e, seg_has_f, f_val)) {
          success = false;
          break;
        }

        // Feed watchdog during long subdivision sequences
        if ((seg % 10) == 0) keepalive();
      }

      if (!success) break;

      // Update tracked position
      track_x = new_x;
      track_y = new_y;
      track_z = new_z;
      track_i = new_i;
      track_j = new_j;
      continue;
    }

    // G28 — copy as-is, reset tracked position
    if (is_g && cmd_num == 28) {
      if (!write_line(line_buf)) { success = false; break; }
      track_x = track_y = track_z = 0;
      track_i = 0;
      track_j = 0;
      continue;
    }

    // G90/G91 — copy and track absolute/relative mode
    if (is_g && cmd_num == 90) {
      if (!write_line(line_buf)) { success = false; break; }
      absolute_mode = true;
      continue;
    }
    if (is_g && cmd_num == 91) {
      if (!write_line(line_buf)) { success = false; break; }
      absolute_mode = false;
      continue;
    }

    // G92 — copy and update tracked position
    if (is_g && cmd_num == 92) {
      if (!write_line(line_buf)) { success = false; break; }
      float val;
      if (parse_gcode_value(line_buf, 'X', val)) track_x = val;
      if (parse_gcode_value(line_buf, 'Y', val)) track_y = val;
      if (parse_gcode_value(line_buf, 'Z', val)) track_z = val;
      if (parse_axis4(line_buf, val)) track_i = val;
      if (parse_axis5(line_buf, val)) track_j = val;
      continue;
    }

    // Everything else (M-codes, other G-codes, etc.) — copy as-is
    if (!write_line(line_buf)) { success = false; break; }
  }

  // Sync and close temp file
  tempfile.sync();
  tempfile.close();

  // Restore IK flags
  tool_centerpoint_control = saved_tcpc;
  #if ENABLED(CALIBRATION_CORRECTION)
    calibration_for_move_target = saved_cal;
  #endif

  if (!success) {
    SERIAL_ERROR_MSG("M668: Write error during processing");
    card.removeFile(IK_TEMP_FILENAME);
    return false;
  }

  // Close source file and open temp file for printing
  card.closefile();
  card.openFileRead(IK_TEMP_FILENAME);

  if (!card.isFileOpen()) {
    SERIAL_ERROR_MSG("M668: Failed to open processed file");
    return false;
  }

  // openFileRead() calls abortFilePrintNow() which clears the printing flag,
  // so we must restart it
  card.startOrResumeFilePrinting();

  CardReader::ik_temp_file_active = true;
  SERIAL_ECHOLNPGM("M668: IK pre-processing complete (", line_count, " lines)");
  ui.set_status(F("IK ready"));

  return true;
}

#endif // IK_PREPROCESS
