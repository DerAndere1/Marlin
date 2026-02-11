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
#include "../MarlinCore.h"
#include <stdlib.h>  // for dtostrf (float-to-string without sprintf %f)

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

// Read one line from a given file handle.
// Returns false at EOF.
static bool read_line_from(MediaFile &file, char *buf, uint16_t maxlen) {
  uint16_t i = 0;
  while (i < maxlen - 1) {
    const int16_t c = file.read();
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

// Append a dtostrf-formatted float to buf at position pos. Returns new pos.
// Uses dtostrf instead of sprintf %f because STM32 newlib-nano doesn't support float printf.
static int append_float(char *buf, int pos, const float val, const uint8_t prec) {
  char tmp[16];
  dtostrf(val, 1, prec, tmp);  // width=1 = minimum width, no padding
  const int len = strlen(tmp);
  memcpy(buf + pos, tmp, len);
  return pos + len;
}

// Format and write a transformed G1 line using static out_buf.
// Uses dtostrf for float formatting (sprintf %f is broken on STM32 newlib-nano).
static bool write_transformed_line(
  const bool is_g0,
  const abce_pos_t &transformed,
  const float i_val, const float j_val,
  const bool has_e, const float e_val,
  const bool has_f, const float f_val
) {
  int pos = 0;
  out_buf[pos++] = 'G';
  out_buf[pos++] = is_g0 ? '0' : '1';
  out_buf[pos++] = ' '; out_buf[pos++] = 'X';
  pos = append_float(out_buf, pos, transformed.x, 3);
  out_buf[pos++] = ' '; out_buf[pos++] = 'Y';
  pos = append_float(out_buf, pos, transformed.y, 3);
  out_buf[pos++] = ' '; out_buf[pos++] = 'Z';
  pos = append_float(out_buf, pos, transformed.z, 3);
  out_buf[pos++] = ' '; out_buf[pos++] = AXIS4_NAME;
  pos = append_float(out_buf, pos, i_val, 3);
  out_buf[pos++] = ' '; out_buf[pos++] = AXIS5_NAME;
  pos = append_float(out_buf, pos, j_val, 3);
  if (has_e) {
    out_buf[pos++] = ' '; out_buf[pos++] = 'E';
    pos = append_float(out_buf, pos, e_val, 5);
  }
  if (has_f) {
    out_buf[pos++] = ' '; out_buf[pos++] = 'F';
    pos = append_float(out_buf, pos, f_val, 0);
  }
  out_buf[pos] = '\0';
  return write_line(out_buf);
}

bool preprocess_ik_file() {
  if (!card.isMounted() || !card.isFileOpen()) return false;

  const uint32_t source_size = card.getFileSize();
  const uint32_t start_pos = card.getIndex();

  // Open a SEPARATE file handle for reading the source.
  // This avoids conflicts with card.myfile which idle() may touch
  // via get_sdcard_commands() during idle_no_sleep().
  static MediaFile srcfile;
  if (!srcfile.open(&card.getWorkDir(), card.filename, O_READ)) {
    SERIAL_ERROR_MSG("M668: Failed to open source file for reading");
    return false;
  }
  srcfile.seekSet(start_pos);

  // Prevent get_sdcard_commands() from reading card.myfile during processing.
  // Use sdprintdone flag instead of pauseSDPrint() to avoid triggering
  // the LCD "paused for user" state.
  card.flag.sdprintdone = true;

  // Open temp file for writing
  if (!tempfile.open(&card.getWorkDir(), IK_TEMP_FILENAME, O_CREAT | O_WRITE | O_TRUNC)) {
    SERIAL_ERROR_MSG("M668: Failed to create temp file");
    srcfile.close();
    return false;
  }

  ui.set_status(F("Processing IK..."));
  SERIAL_ECHOLNPGM("M668: IK pre-processing started, source size=", source_size,
                    " start_pos=", start_pos);

  // Write G49 as first line to disable TCPC in the processed file
  if (!write_line("G49")) {
    SERIAL_ERROR_MSG("M668: Write error");
    tempfile.close();
    srcfile.close();
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
  float track_e = 0;                             // Current absolute E position
  bool absolute_mode = true;
  bool relative_e = false;                       // M83 = relative extrusion
  bool success = true;
  uint32_t line_count = 0;
  uint32_t ik_line_count = 0;                    // Output lines transformed through IK
  uint32_t passthrough_count = 0;                // Output lines copied as-is

  while (read_line_from(srcfile, line_buf, IK_LINE_MAXLEN)) {
    line_count++;

    // Keep system alive (watchdog, heaters, serial, UI)
    marlin.idle_no_sleep();

    // Progress update every 500 lines
    if ((line_count % 500) == 0) {
      const uint8_t pct = (uint8_t)((uint32_t)srcfile.curPosition() * 100 / source_size);
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

    // Track M82/M83 extrusion mode
    const bool is_m = (line_buf[0] == 'M' || line_buf[0] == 'm');
    if (is_m) {
      const int m_cmd = atoi(line_buf + 1);
      if (m_cmd == 82) relative_e = false;
      if (m_cmd == 83) relative_e = true;
    }

    // G43.4 — skip (replaced by G49 at top)
    if (is_g && strstr(line_buf, "G43.4")) continue;
    // G49 — skip (already written at top)
    if (is_g && cmd_num == 49) continue;

    // G0 / G1 — transform through IK (only when rotation is involved)
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

      // When no rotation is involved (current and target both zero),
      // IK is identity — copy the command as-is to preserve original
      // coordinates (avoids position tracking issues with G28/G91/G92).
      const bool has_rotation = (new_i != 0 || new_j != 0 || track_i != 0 || track_j != 0);
      if (!has_rotation) {
        if (!write_line(line_buf)) { success = false; break; }
        passthrough_count++;
        track_x = new_x;
        track_y = new_y;
        track_z = new_z;
        track_i = new_i;
        track_j = new_j;
        if (has_e) { if (relative_e) track_e += e_val; else track_e = e_val; }
        continue;
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

        // E subdivision: per-segment delta for M83, interpolated absolute for M82
        float seg_e;
        if (!has_e) {
          seg_e = 0;
        } else if (relative_e) {
          seg_e = e_val / (float)segments;  // Equal per-segment delta
        } else {
          seg_e = track_e + (e_val - track_e) * t;  // Interpolate absolute position
        }

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

        ik_line_count++;

        // Debug: print first 10 IK-transformed output lines
        if (ik_line_count <= 10) {
          SERIAL_ECHOPGM("M668 IK[");
          SERIAL_ECHO(ik_line_count);
          SERIAL_ECHOPGM("]: ");
          SERIAL_ECHOLN(out_buf);
        }

        // Feed watchdog during long subdivision sequences
        if ((seg % 10) == 0) marlin.idle_no_sleep();
      }

      if (!success) break;

      // Update tracked position
      track_x = new_x;
      track_y = new_y;
      track_z = new_z;
      track_i = new_i;
      track_j = new_j;
      if (has_e) { if (relative_e) track_e += e_val; else track_e = e_val; }
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
      if (parse_gcode_value(line_buf, 'E', val)) track_e = val;

      // Debug: show G92 state changes
      SERIAL_ECHOPGM("M668 G92: track_i=");
      SERIAL_ECHO(track_i);
      SERIAL_ECHOPGM(" track_j=");
      SERIAL_ECHO(track_j);
      SERIAL_ECHOPGM(" at line ");
      SERIAL_ECHOLN(line_count);
      continue;
    }

    // Everything else (M-codes, other G-codes, etc.) — copy as-is
    if (!write_line(line_buf)) { success = false; break; }
  }

  // Close source and temp files
  srcfile.close();
  tempfile.sync();
  tempfile.close();

  // Restore IK flags
  tool_centerpoint_control = saved_tcpc;
  #if ENABLED(CALIBRATION_CORRECTION)
    calibration_for_move_target = saved_cal;
  #endif

  if (!success) {
    SERIAL_ERROR_MSG("M668: Write error during processing");
    card.flag.sdprintdone = false;  // Restore SD state
    card.removeFile(IK_TEMP_FILENAME);
    return false;
  }

  // NOTE: Do NOT call queue.clear() here. M668 runs inside queue.advance(),
  // and clearing the ring buffer during advance causes uint8_t underflow
  // (length 0 -> 255), corrupting the queue. The few buffered start-gcode
  // commands (e.g. G28) are harmless to re-execute from the temp file.

  // Close source file (card.myfile) and open temp file for printing
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
  SERIAL_ECHOLNPGM("M668: IK pre-processing complete (", line_count, " lines, ",
                    card.getFileSize(), " bytes)");
  SERIAL_ECHOLNPGM("M668: IK-transformed: ", ik_line_count,
                    " passthrough: ", passthrough_count,
                    " relative_e: ", relative_e ? "yes" : "no");
  ui.set_status(F("IK ready"));

  return true;
}

#endif // IK_PREPROCESS
