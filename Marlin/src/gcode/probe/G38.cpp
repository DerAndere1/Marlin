/**
 * Marlin2ForPipetBot [https://github.com/DerAndere1/Marlin]
 * Copyright 2019 - 2026 DerAndere and other Marlin2ForPipetBot authors [https://github.com/DerAndere1/Marlin]
 *
 * Based on:
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 - 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(G38_PROBE_TARGET)

#include "../gcode.h"

#include "../../module/endstops.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../lcd/marlinui.h"

probe_target_t G38_move{0};

inline bool G38_single_probe(const uint8_t move_value) {
  #if ENABLED(BLTOUCH)
    // Ensure the BLTouch is deployed. (Does nothing if already deployed.)
    // Don't deploy with high_speed_mode enabled. The probe already re-deploys itself.
    if ((!bltouch.high_speed_mode) && bltouch.deploy())
      return true;
  #endif

  #if HAS_Z_SERVO_PROBE && (ENABLED(Z_SERVO_INTERMEDIATE_STOW) || defined(Z_SERVO_MEASURE_ANGLE))
    probe.probe_specific_action(true);  // Always re-deploy in this case
  #endif
  #if ENABLED(SOLENOID_PROBE)
    if (probe.deploy()) {
      endstops.not_homing();
      return true;
    }
  #endif
  endstops.enable(true);
  G38_move.type = move_value;
  motion.prepare_line_to_destination();
  planner.synchronize();
  G38_move.type = 0;
  #if ENABLED(SOLENOID_PROBE) || ALL(HAS_Z_SERVO_PROBE, Z_SERVO_INTERMEDIATE_STOW)
    probe.probe_specific_action(false);  //  Always stow
  #endif
  endstops.hit_on_purpose();
  motion.set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  motion.sync_plan_position();
  return false;
}

/**
 * Handle G38.N where N is the sub-code for the type of probe:
 *  2 - Probe toward workpiece, stop on contact, signal error if failure
 *  3 - Probe toward workpiece, stop on contact
 *  4 - Probe away from workpiece, stop on contact break, signal error if failure
 *  5 - Probe away from workpiece, stop on contact break
 */
FORCE_INLINE bool G38_run_probe() {

  bool G38_pass_fail = false;

  #if MULTIPLE_PROBING > 1
    // Get direction of move and retract
    xyz_float_t retract_mm;
    LOOP_NUM_AXES(i) {
      const float dist = destination[i] - current_position[i];
      retract_mm[i] = ABS(dist) < G38_MINIMUM_MOVE ? 0 : home_bump_mm((AxisEnum)i) * (dist > 0 ? -1 : 1);
    }
  #endif

  planner.synchronize(); // Wait until the machine is idle

   // Move flag value
  #if ENABLED(G38_PROBE_AWAY)
    constexpr uint8_t move_value = 1;
  #endif

  // Move flag value
  #if ENABLED(G38_PROBE_AWAY)
    const uint8_t move_value = parser.subcode;
  #else
    constexpr uint8_t move_value = 1;
  #endif

  G38_move.triggered = false;

  // Move until destination reached or target hit
  if (G38_single_probe(move_value)) {
    G38_pass_fail = false;
    return false;
  }

  if (G38_move.triggered) {

    G38_pass_fail = true;

    #if MULTIPLE_PROBING > 1
      // Move away by the retract distance
      destination = current_position + retract_mm;
      endstops.enable(false);
      prepare_line_to_destination();
      planner.synchronize();

      REMEMBER(fr, feedrate_mm_s, feedrate_mm_s * 0.25);

      // Bump the target more slowly
      destination -= retract_mm * 2;

      if (G38_single_probe(move_value)) return false;
    #endif
  
    // Report a good probe result in machine coordinate system to the host and LCD
    const xyz_pos_t probe_pos = (TERN1(HAS_TOOL_LENGTH_COMPENSATION, motion.simple_tool_length_compensation) || TERN0(HAS_TOOL_CENTERPOINT_CONTROL, motion.tool_centerpoint_control)) ? motion.position + DIFF_TERN(HAS_HOTEND_OFFSET, probe.offset, motion.hotend_offset[motion.extruder]) : motion.position + probe.offset;
    SString<30> msg(
      F("Machine X:"), p_float_t(probe_pos.x, 2),
      F(" Y:"), p_float_t(probe_pos.y, 2),
      F(" Z:"), p_float_t(probe_pos.z, 3)
    );
    msg.echoln();
    TERN_(VERBOSE_SINGLE_PROBE, ui.set_status(msg));
  }

  endstops.not_homing();
  return G38_pass_fail;
}

/**
 *
 *  G38.4 - Probe away from workpiece, stop on contact break, signal error if failure
 *  G38.5 - Probe away from workpiece, stop on contact break
 *
 * Parameters:
 *
 *   X   Probe X position (default current X)
 *   Y   Probe Y position (default current Y)
 *   Z   Probe Z position (default current Z)
 *   S   Stow the probe after probing (default: 0)
 */
void GcodeSuite::G38(const int8_t subcode) {

  // Get X Y Z E F
  get_destination_from_command();

  motion.remember_feedrate_scaling_off();

  const bool error_on_fail = TERN(G38_PROBE_AWAY, !TEST(subcode, 0), subcode == 2);

  // If any axis has enough movement, do the move
  LOOP_NUM_AXES(i) {
    if (ABS(motion.destination[i] - motion.position[i]) >= G38_MINIMUM_MOVE) {
      if (!parser.seenval('F')) motion.feedrate_mm_s = motion.homing_feedrate((AxisEnum)i);
      // If G38.2 fails throw an error
      if (!G38_run_probe() && error_on_fail) SERIAL_ERROR_MSG("Failed to reach target");
      break;
    }
  }

  motion.restore_feedrate_and_scaling();
};
#endif // G38_PROBE_TARGET