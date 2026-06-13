/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

//
// Tool Change Menu
//

#include "../../inc/MarlinConfigPre.h"
#include "../../MarlinCore.h"

#if ALL(HAS_MARLINUI_MENU, MANUAL_SWITCHING_TOOLHEAD)

#include "menu.h"
#include "menu_item.h"
#include "../../module/tool_change.h"
#include "../../feature/pause.h"
#include "../../gcode/queue.h"

/**
 * Inject a toolchange gcode (Tn) using the editable.uint8 field,
 * and then return to the status screen.
 */
inline void inject_toolchange_gcode() {
  char tgc[3] = { '\0' };
  const char n = editable.uint8 + '0';
  sprintf_P(tgc, PSTR("T%c"), n);
  queue.inject(tgc);
  ui.return_to_status();
}

/**
 * Hotend Tool menu, listing all tools set up in Configuration.h.
 */
void menu_tool_change_tool() {
  START_MENU();

  //
  // ^ Tool Change
  //
  BACK_ITEM(MSG_TOOL_CHANGE);

  // Display Hotend 1 .. Hotend n, or hotend names.
  for (uint8_t e = 0; e < TOOLS; ++e) {
    editable.uint8 = e;
    if (marlin.printingIsActive()) {
      CONFIRM_ITEM_F(F("Change Tool?"),
        MSG_YES, MSG_NO,
        inject_toolchange_gcode, ui.goto_previous_screen,
        F("Change Tool?"), F(""), F("?"));
    }
    else
      ACTION_ITEM_F(tool_name(e), inject_toolchange_gcode);
  }

  END_MENU();
}

/**
 * Main "Tool Change" menu, with options for Hotends,
 * Laser/Spindle, and unpowered tools.
 */
void menu_tool_change() {
  START_MENU();

  // display the current tool
  STATIC_ITEM_F(tool_name(motion.extruder), SS_DEFAULT|SS_INVERT);

  BACK_ITEM(MSG_MAIN_MENU);

  #if HAS_MULTI_TOOLS
      SUBMENU(MSG_TOOL, menu_tool_change_tool);
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU && MANUAL_SWITCHING_TOOLHEAD