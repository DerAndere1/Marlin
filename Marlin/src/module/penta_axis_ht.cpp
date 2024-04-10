/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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


/**
 * @file penta_axis_ht.cpp
 * @author DerAndere
 * @brief Kinematics for a 5 axis CNC machine in head-table configuration.
 *
 * This machine has a tilting head and a horizontal rotary table.
 *
 * Copyright 2024 DerAndere
 *
 * Based on a relicensed verion of LinuxCNC file maxkins.c
 * Author: Chris Radek <chris@timeguy.com>
 *
 * Copyright (c) 2007, 2022 Chris Radek
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(PENTA_AXIS_HT)

#include "penta_axis_ht.h"
#include "motion.h"


// Initialized by settings.load()
float segments_per_second;
float mrzp_offset_z;

/**
 * penta axis head table inverse kinematics
 *
 * Calculate the joints positions for a given position, storing the result in the global delta[] array.
 * The raw position is interpreted as native machine position using native_to_joint().
 */
void inverse_kinematics(const xyz_pos_t &raw) {
    delta = native_to_joint(raw);
}

/**
 * Calculate the joints positions for a given position.
 *
 * This is an expensive calculation.
 */
xyz_pos_t native_to_joint(const xyz_pos_t &native) {
  if (!tool_centerpoint_control) return native;

  // X and Y hotend offsets must be applied in Cartesian space with no "spoofing"
  xyz_pos_t pos = NUM_AXIS_ARRAY(
                    DIFF_TERN(HAS_HOTEND_OFFSET, native.x, hotend_offset[active_extruder].x),
                    DIFF_TERN(HAS_HOTEND_OFFSET, native.y, hotend_offset[active_extruder].y),
                    native.z,
                    native.i,
                    native.j
                  );

  const float pivot_length = DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_z, hotend_offset[active_extruder].z);
  const float i_rad = RADIANS(pos.i);

  #if HAS_J_AXIS || AXIS4_NAME == 'B'
    // B correction
    const float zb = pivot_length * cos(i_rad) - mrzp_offset_z;
    const float xb = pivot_length * sin(i_rad);

    #if HAS_J_AXIS
      // C correction
      const float xyr = HYPOT(pos.x, pos.y);
      const float xytheta = ATAN2(pos.y, pos.x) - RADIANS(pos.j);
    #endif

    const xyz_pos_t joints_pos = NUM_AXIS_ARRAY(
      TERN(HAS_J_AXIS, xyr * cos(xytheta) + xb, pos.x + xb),
      TERN(HAS_J_AXIS, xyr * sin(xytheta), pos.y),
      pos.z + zb,
      pos.i,
      pos.j
    );

  #elif (!HAS_J_AXIS) && (AXIS4_NAME == 'C')
    // C correction
    const float xyr = HYPOT(pos.x, pos.y);
    const float xytheta = ATAN2(pos.y, pos.x) - i_rad;

    const xyz_pos_t joints_pos = NUM_AXIS_ARRAY(
      xyr * cos(xytheta),
      xyr * sin(xytheta),
      DIFF_TERN(HAS_HOTEND_OFFSET, pos.z, hotend_offset[active_extruder].z),
      pos.i,
      pos.j
    );
  #endif

  return joints_pos;
}

#endif //PENTA_AXIS_HT
