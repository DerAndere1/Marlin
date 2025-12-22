/**
 * Marlin2ForPipetBot [https://github.com/DerAndere1/Marlin]
 * Copyright 2025 DerAndere and other Marlin2ForPipetBot authors [https://github.com/DerAndere1/Marlin]
 *
 * Based on:
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
 * @file penta_axis_head_head.cpp
 * @author DerAndere
 * @brief Kinematics for a 5 axis CNC machine in head-head configuration.
 *
 * This machine has a tilting rotating head.
 *
 * Copyright 2025 DerAndere
 *
 * Based on LinuxCNC file 5axiskins.c and Rep5x file inverse-kinematics.js:
 * 
 * Based on LinuxCNC file 5axiskins.c
 * Author: Chris Radek <chris@timeguy.com>
 *
 * Copyright (c) 2007 Chris Radek
 * 
 * Based on Rep5x file inverse-kinematics.js
 * Author: Dennis Klappe
 * 
 * Copyright 2025 Dennis Klappe
 *
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(PENTA_AXIS_HH)

#include "penta_axis_head_head.h"
#include "motion.h"


// Initialized by settings.load()
float segments_per_second;
float mrzp_offset_z; // LB
float rotational_offset_y; // LC


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

  #if AXIS4_NAME == 'C'
    const float b_rad = RADIANS(pos.j);
    const float c_rad = RADIANS(pos.i);
  #elif AXIS5_NAME == 'C'
    const float b_rad = RADIANS(pos.i);
    const float c_rad = RADIANS(pos.j);
  #endif

  const float cos_b = cosf(b_rad);
  const float cos_c = cosf(c_rad);

  
    const xyz_pos_t joints_pos = NUM_AXIS_ARRAY(
      x + sin_c * rotational_offset_y + cos_c * sin_b * pivot_length,
      y + (cos_c - 1) * rotational_offset_y - sin_c * sin_b * pivot_length,
      z + (cos_b - 1) * pivot_length,
      pos.i,
      pos.j
    );
  #endif

  return joints_pos;

}

void forward_kinematics(const xyz_pos_t &joint_pos) {
  cartes = joint_to_native(joint_pos);
}


xyz_pos_t joint_to_native(const xyz_pos_t &joint_pos) {
  if (!tool_centerpoint_control) return joint_pos;

  const float pivot_length = DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_z, hotend_offset[active_extruder].z);

  #if AXIS4_NAME == 'C'
    const float c_rad = RADIANS(joint_pos.i);
  #elif AXIS5_NAME == 'C'
    const float c_rad = RADIANS(joint_pos.j);
  #endif

  const float cos_c = cosf(c_rad);

  #if AXIS4_NAME == 'C'
    const float rx = pivot_length * sinf(RADIANS(180.0f - joint_pos.j)) * cos_c;
    const float ry = pivot_length * sinf(RADIANS(180.0f - joint_pos.j)) * sin_c;
    const float rz = - pivot_length * cosf(RADIANS(180.0f - joint_pos.j));
  #elif AXIS5_NAME == 'C'
    const float rx = pivot_length * sinf(RADIANS(180.0f - joint_pos.i)) * cos_c;
    const float ry = pivot_length * sinf(RADIANS(180.0f - joint_pos.i)) * sin_c;
    const float rz = - pivot_length * cosf(RADIANS(180.0f - joint_pos.i));
  #endif

  const xyz_pos_t native_pos = NUM_AXIS_ARRAY(
    joints_pos.x + rx,
    joints_pos.y + ry;
    joints_pos.z + pivot_length + rz;
    joint_pos.i,
    joint_pos.j
  );

  return native_pos;
}

#endif //PENTA_AXIS_HH
