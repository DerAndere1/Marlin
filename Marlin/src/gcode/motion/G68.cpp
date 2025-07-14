/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(ROTATE_WORKSPACE)

  #include "../gcode.h"
  #include "../../module/motion.h"
  
  /**
   * G68: Set Workspace Rotation
   *
   * Set the rotation (about Z axis) for the current workspace (begins at 0).
   *
   * Parameters:
   *   P<index>  Workspace index (Optional, default: current)
   *   R<deg>    Rotation angle in degrees (Required)
   *
   * Example:
   *   G68 R45          ; Rotate current workspace by 45° counter-clockwise (when viewed from positive Z) 
   *                    ; around current position
   *   G68 P2 R-30      ; Rotate workspace 2 by -30° around current position
   *   G68 P2 X0 Y0 R45 ; Rotate workspace 2 by 45°C around X0 Y0 (X and Y are specified in the current workspace)
   *
   * NOTES:
   *   - Only rotation is set. No translation/offset is changed.
   *   - All subsequent moves are rotated by the specified angle.
   */


  extern xyze_pos_t destination;

  extern xyz_pos_t raw_destination;


  void GcodeSuite::G68() {
  
  const uint8_t index = parser.seenval('P') ? parser.value_byte() : 0;
  const int8_t target_system = (index == 0) ? active_coordinate_system : (index - 1);  // P0 selects current coordinate system. P1 is G54, which is Marlin coordinate_system 0 

  if (!parser.seenval('R')) {
    SERIAL_ECHOLNPGM("Missing R parameter (rotation angle).");
    return;
  }
  else {
    const float r = parser.value_float();
    if (!WITHIN(target_system, 0, MAX_COORDINATE_SYSTEMS - 1)) {
      SERIAL_ECHOLNPGM("Invalid workspace index.");
      return;
    }
    else {
      rotation_angle[target_system] = r;
      SERIAL_ECHOLNPGM("Rotation for workspace ", target_system, " set to ", r, " degrees.");
    }

    #if HAS_X_AXIS
      rotation_center_x = parser.seenval('X') ? LOGICAL_TO_NATIVE(parser.value_axis_units(X_AXIS), X_AXIS) : current_position.x;
    #endif

    #if HAS_Y_AXIS
      rotation_center_y = parser.seenval('Y') ? LOGICAL_TO_NATIVE(parser.value_axis_units(Y_AXIS), Y_AXIS) : current_position.y;
    #endif
    }
  }
#endif // ROTATE_WORKSPACE
