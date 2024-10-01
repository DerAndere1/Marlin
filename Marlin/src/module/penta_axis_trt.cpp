/**
 * Marlin2ForPipetBot [https://github.com/DerAndere1/Marlin]
 * Copyright 2019 - 2024 DerAndere and other Marlin2ForPipetBot authors [https://github.com/DerAndere1/Marlin]
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
 * @file penta_axis_trt.cpp
 * @author DerAndere
 * @brief Kinematics for a 5 axis CNC machine in tilting rotary table configuration.
 *
 *  A) XYZAC_TRT:
 *     This machine has a tilting table (A axis parallel to the X axis) and horizontal rotary
 *     mounted to the table (C axis).
 *  B) XYZBC_TRT:
 *     This machine has a tilting table (B axis parallel to the Y axis) and horizontal rotary
 *     mounted to the table (C axis).
 *
 * Copyright 2024 DerAndere
 *
 * Based on the file trtfuncs.c from LinuxCNC (https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/kinematics/trtfuncs.c)
 * Copyright 2016 Rudy du Preez <rudy@asmsa.co.za>
 * Author: Rudy du Preez <rudy@asmsa.co.za>
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(PENTA_AXIS_TRT)

#include "penta_axis_trt.h"
#include "motion.h"

// Initialized by settings.load()
float segments_per_second;
float mrzp_offset_x;
float mrzp_offset_y;
float mrzp_offset_z;
float rotational_offset_x;
float rotational_offset_y;
float rotational_offset_z;

/**
 * 5 axis tilting rotary table inverse kinematics
 *
 * Calculate the joints positions for a given position, storing the result in the global delta[] array.
 * The raw position is interpreted as machine position using native_to_joint().
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

  const float pivot_length_x = native.x - mrzp_offset_x;
  const float pivot_length_y = native.y - mrzp_offset_y;
  const float pivot_length_z = native.z - mrzp_offset_z;

  const float i_rad = RADIANS(native.i);
  const float cos_i = cos(i_rad);
  const float sin_i = sin(i_rad);

  const float j_rad = TERN0(HAS_J_AXIS, RADIANS(native.j));
  const float cos_j = TERN1(HAS_J_AXIS, cos(j_rad));
  const float sin_j = TERN0(HAS_J_AXIS, sin(j_rad));

  #if AXIS4_NAME == 'A'
    // computed position
    const xyz_pos_t joints_pos = NUM_AXIS_ARRAY(
        cos_j      * pivot_length_x
      - sin_j      * pivot_length_y
      + DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_x, hotend_offset[active_extruder].x),

        sin_j * cos_i * pivot_length_x
      + cos_j * cos_i * pivot_length_y
      +         sin_i * pivot_length_z
      -         cos_i * rotational_offset_y
      -         sin_i * rotational_offset_z
      + rotational_offset_y
      + DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_y, hotend_offset[active_extruder].y),

      - sin_j * sin_i * pivot_length_x
      - cos_j * sin_i * pivot_length_y
      +         cos_i * pivot_length_z
      +         sin_i * rotational_offset_y
      -         cos_i * rotational_offset_z
      + rotational_offset_z      
      + DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_z, hotend_offset[active_extruder].z),

        native.i,

        native.j
    );
  #elif AXIS4_NAME == 'B'
    // computed position
    const xyz_pos_t joints_pos = NUM_AXIS_ARRAY(
        cos_j * cos_i * pivot_length_x
      + sin_j * cos_i * pivot_length_y
      -         sin_i * pivot_length_z
      -         cos_i * rotational_offset_x 
      +         sin_i * rotational_offset_z 
      + rotational_offset_x 
      + DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_x, hotend_offset[active_extruder].x),

      - sin_j      * pivot_length_x
      + cos_j      * pivot_length_y
      + DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_y, hotend_offset[active_extruder].y),

        cos_j * sin_i * pivot_length_x
      + sin_j * sin_i * pivot_length_y
      +         cos_i * pivot_length_z
      -         sin_i * rotational_offset_x 
      -         cos_i * rotational_offset_z
      + rotational_offset_z
      + DIFF_TERN(HAS_HOTEND_OFFSET, mrzp_offset_z, hotend_offset[active_extruder].z),

        native.i,

        native.j
    );
  #endif
  return joints_pos;
}

#endif // PENTA_AXIS_TRT
