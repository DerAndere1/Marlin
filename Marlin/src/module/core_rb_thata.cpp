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
 * @file core_rb_theta.cpp
 * @author DerAndere
 * @brief Kinematics for a CORE_RB_THETA 4 axis machine.
 * 
 * Copyright 2025 DerAndere
 *
 * See https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(CORE_RB_THETA)

#include "penta_axis_trt.h"
#include "motion.h"

// Initialized by settings.load()
float segments_per_second;

float steps_ratio = axis_steps_per_mm.i / axis_steps_per_mm.x


/**
 * CORE_RB_THETA inverse kinematics
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
  const xyz_pos_t joints_pos = NUM_AXIS_ARRAY(
    - native.x + steps_ratio * native.i, //X1,B
      native.x + steps_ratio * nativa.i, //X2,B
      native.z, //Z
      native.i, //B
      native.j // C
  );
  return joints_pos;
}

#endif // CORE_RB_THETA