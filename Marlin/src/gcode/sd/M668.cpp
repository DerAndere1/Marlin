/**
 * Rep5x IK Pre-processor G-code
 * Copyright 2025-2026 Dennis Klappe
 *
 * M668: Pre-process remaining G-code file through inverse kinematics
 *
 * Usage:
 *   M668          - Pre-process remaining file through IK
 *   M668 B<temp>  - Also start heating bed to <temp>
 *   M668 H<temp>  - Also start heating hotend to <temp>
 *
 * Place in start G-code before G28. The command:
 *   1. Starts preheating (B/H params) so heatup overlaps with processing
 *   2. Reads remaining G-code lines from the open file
 *   3. Transforms G0/G1 moves through inverse kinematics
 *   4. Writes transformed output to _iktmp.gco with G49 (TCPC off)
 *   5. Switches to printing the processed file
 *
 * On failure, printing continues with the original file and real-time IK.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(IK_PREPROCESS)

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../sd/ik_preprocessor.h"
#include "../../module/temperature.h"

void GcodeSuite::M668() {
  if (!card.isFileOpen()) {
    SERIAL_ERROR_MSG("M668: No file open");
    return;
  }

  // Start preheating immediately so it overlaps with processing time
  if (parser.seenval('B'))
    thermalManager.setTargetBed(parser.value_celsius());
  if (parser.seenval('H'))
    thermalManager.setTargetHotend(parser.value_celsius(), 0);

  // Pre-process remaining file through IK
  if (!preprocess_ik_file()) {
    SERIAL_ERROR_MSG("M668: IK pre-processing failed, falling back to real-time IK");
  }
}

#endif // IK_PREPROCESS
