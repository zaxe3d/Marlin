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

#include "../../inc/MarlinConfig.h"
#include "../../lcd/extui/lib/dgus/fysetc/DGUSDisplayDef.h" //Elsan
#include "../../lcd/extui/lib/dgus/DGUSScreenHandler.h"

extern DGUSScreenHandler ScreenHandler; //Elsan

#if ENABLED(SDSUPPORT)

#include "../gcode.h"
#include "../../sd/cardreader.h"

#if ENABLED(PRINTER_EVENT_LEDS)
  #include "../../feature/leds/printer_event_leds.h"
#endif

/**
 * M524: Abort the current SD print job (started with M24)
 */
void GcodeSuite::M524() {

  if (IS_SD_PRINTING())
    card.flag.abort_sd_printing = true;
  else if (/*card.isMounted()*/1)
    card.closefile();

#if ENABLED(PRINTER_EVENT_LEDS)
  printerEventLEDs.onPrintAborted();
#endif

  ScreenHandler.GotoScreen(DGUSLCD_SCREEN_MAIN);  //Elsan DGUS screen not updated after abort from XDesktop.
}

#endif // SDSUPPORT
