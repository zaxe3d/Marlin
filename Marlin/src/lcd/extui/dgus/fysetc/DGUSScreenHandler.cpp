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

#include "../../../../inc/MarlinConfigPre.h"

#if ENABLED(DGUS_LCD_UI_FYSETC)

#include "../DGUSScreenHandler.h"
#include "DGUSDisplayDef.h" //Elsan

#include "../../../../MarlinCore.h"
#include "../../../../gcode/queue.h"
#include "../../../../libs/duration_t.h"
#include "../../../../module/settings.h"
#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../module/planner.h"
#include "../../../../module/printcounter.h"
#include "../../../../sd/cardreader.h"

extern char buf_main[][200];
extern int file_cnt3;

/*typedef struct  {
    ExtUI::extruder_t extruder; // which extruder to operate
    uint8_t action; // load or unload
    uint8_t target_temp; // accepted target temp  //Elsan
    uint8_t material_type;  //Elsan
    bool heated; // heating done ?
    float purge_length; // the length to extrude before unload, prevent filament jam
    float processed_length; //Elsan
    xyze_pos_t resume_position; //Elsan
    bool waiting_confirmation;  //Elsan
  } filament_data_t;*/
extern filament_data_t filament_data;

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../../feature/powerloss.h"
#endif

#if ENABLED(SDSUPPORT)

  extern ExtUI::FileList filelist;

  /*void DGUSScreenHandler::DGUSLCD_SD_FileSelected(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t touched_nr = (int16_t)swap16(*(uint16_t*)val_ptr) + top_file;
    if (touched_nr > filelist.count()) return;
    if (!filelist.seek(touched_nr)) return;

    if (filelist.isDir()) {
      filelist.changeDir(filelist.filename());
      top_file = 0;
      ForceCompleteUpdate();
      return;
    }

    #if ENABLED(DGUS_PRINT_FILENAME)
      // Send print filename
      dgusdisplay.WriteVariable(VP_SD_Print_Filename, filelist.filename(), VP_SD_FileName_LEN, true);
    #endif

    // Setup Confirmation screen
    file_to_print = touched_nr;

    HandleUserConfirmationPopUp(VP_SD_FileSelectConfirm, nullptr, PSTR("Print file"), filelist.filename(), PSTR("from SD Card?"), true, true, false, true);
  }*/
  void DGUSScreenHandler::DGUSLCD_SD_FileSelected(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t touched_nr = (int16_t)swap16(*(uint16_t*)val_ptr) + top_file;
    /*
    if (touched_nr > filelist.count()) return;
    if (!filelist.seek(touched_nr)) return;
    if (filelist.isDir()) {
      filelist.changeDir(filelist.filename());
      top_file = 0;
      ForceCompleteUpdate();
      return;
    }
    */

    #if ENABLED(DGUS_PRINT_FILENAME)
      // Send print filename
      dgusdisplay.WriteVariable(VP_SD_Print_Filename, /*filelist.filename()*/buf_main[touched_nr+file_cnt3], VP_SD_FileName_LEN, true);
    #endif

    // Setup Confirmation screen
    file_to_print = touched_nr;
    HandleUserConfirmationPopUp(VP_SD_FileSelectConfirm, nullptr, PSTR("Print file"), /*filelist.filename()*/buf_main[touched_nr+file_cnt3], PSTR("from SD Card?"), true, true, false, true);
  }

  /*void DGUSScreenHandler::DGUSLCD_SD_StartPrint(DGUS_VP_Variable &var, void *val_ptr) {
    if (!filelist.seek(file_to_print)) return;
    ExtUI::printFile(filelist.shortFilename());
    GotoScreen(DGUSLCD_SCREEN_SDPRINTMANIPULATION);
  }*/
  void DGUSScreenHandler::DGUSLCD_SD_StartPrint(DGUS_VP_Variable &var, void *val_ptr) {
    //Elsan from old version
    uint16_t value = swap16(*(uint16_t*)val_ptr);
    if (!value) {
      PopToOldScreen();
      return;
    }
    /////////////////////////////////////
    //if (!filelist.seek(file_to_print)) return;

    //ExtUI::printFile(filelist.shortFilename());
    ExtUI::printFile(buf_main[file_to_print+file_cnt3]);
    ScreenHandler.GotoScreen(
      #if ENABLED(DGUS_LCD_UI_ORIGIN)
        DGUSLCD_SCREEN_STATUS
      #else
        DGUSLCD_SCREEN_SDPRINTMANIPULATION
      #endif
    );
  }

  /*void DGUSScreenHandler::DGUSLCD_SD_ResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr) {

    if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.
    switch (swap16(*(uint16_t*)val_ptr)) {
      case 0: { // Resume
        if (ExtUI::isPrintingFromMediaPaused()) {
          ExtUI::resumePrint();
        }
      } break;

      case 1: // Pause

        GotoScreen(DGUSLCD_SCREEN_SDPRINTMANIPULATION);
        if (!ExtUI::isPrintingFromMediaPaused()) {
          ExtUI::pausePrint();
          //ExtUI::mks_pausePrint();
        }
        break;
      case 2: // Abort
        HandleUserConfirmationPopUp(VP_SD_AbortPrintConfirmed, nullptr, PSTR("Abort printing"), filelist.filename(), PSTR("?"), true, true, false, true);
        break;
    }
  }*/
  void DGUSScreenHandler::DGUSLCD_SD_ResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr) {
    //if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.
    switch (swap16(*(uint16_t*)val_ptr)) {
      case 0:  // Resume
        //if (ExtUI::isPrintingFromMediaPaused()) ExtUI::resumePrint();
        ExtUI::resumePrint(); //Elsan do we really need isPrintingFromMediaPaused?
        //queue.inject_P(PSTR("M24"));  //Elsan working without ADVANCED_PAUSE_FEATURE
        break;
      case 1:  // Pause
        //if (!ExtUI::isPrintingFromMediaPaused()) ExtUI::pausePrint();
        ExtUI::pausePrint();
        //queue.inject_P(PSTR("M25"));  //Elsan working without ADVANCED_PAUSE_FEATURE
        break;
      case 2:  // Abort                    
        ScreenHandler.HandleUserConfirmationPopUp(VP_SD_AbortPrintConfirmed, nullptr, PSTR("Abort printing"), /*filelist.filename()*/buf_main[file_to_print+file_cnt3], PSTR("?"), true, true, false, true);
        break;
      case 3:  // Change filament
        if (!ExtUI::isPrintingFromMediaPaused()) {
          ExtUI::pausePrint();
          GotoScreen(DGUSLCD_SCREEN_SDUTILITY);
        }
        break;  
    }
  }

  /*void DGUSScreenHandler::DGUSLCD_SD_SendFilename(DGUS_VP_Variable& var) {
    uint16_t target_line = (var.VP - VP_SD_FileName0) / VP_SD_FileName_LEN;
    if (target_line > DGUS_SD_FILESPERSCREEN) return;
    char tmpfilename[VP_SD_FileName_LEN + 1] = "";
    var.memadr = (void*)tmpfilename;

    if (filelist.seek(top_file + target_line)) {
      snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s%c"), filelist.filename(), filelist.isDir() ? '/' : 0); // snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s"), filelist.filename());
    }
    DGUSLCD_SendStringToDisplay(var);
  }*/
  void DGUSScreenHandler::DGUSLCD_SD_SendFilename(DGUS_VP_Variable& var) {
    uint16_t target_line = (var.VP - VP_SD_FileName0) / VP_SD_FileName_LEN;
    if (target_line > DGUS_SD_FILESPERSCREEN) return;
    char tmpfilename[VP_SD_FileName_LEN + 1] = "";
    var.memadr = (void*)tmpfilename;
    if (filelist.seek(top_file + target_line))
      snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s%c"), filelist.filename(), filelist.isDir() ? '/' : 0);
    DGUSLCD_SendStringToDisplay(var);
  }

  /*void DGUSScreenHandler::SDCardInserted() {
    top_file = 0;
    filelist.refresh();
    auto cs = getCurrentScreen();
    if (cs == DGUSLCD_SCREEN_MAIN || cs == DGUSLCD_SCREEN_STATUS)
      GotoScreen(DGUSLCD_SCREEN_SDFILELIST);
  }*/
  void DGUSScreenHandler::SDCardInserted() {
    top_file = 0;
    filelist.refresh(); 
    auto cs = ScreenHandler.getCurrentScreen();
    if (cs == DGUSLCD_SCREEN_MAIN || cs == DGUSLCD_SCREEN_STATUS)
      ScreenHandler.GotoScreen(DGUSLCD_SCREEN_SDFILELIST);
  }

  /*void DGUSScreenHandler::SDCardRemoved() {
    if (current_screen == DGUSLCD_SCREEN_SDFILELIST
        || (current_screen == DGUSLCD_SCREEN_CONFIRM && (ConfirmVP == VP_SD_AbortPrintConfirmed || ConfirmVP == VP_SD_FileSelectConfirm))
        || current_screen == DGUSLCD_SCREEN_SDPRINTMANIPULATION
    ) GotoScreen(DGUSLCD_SCREEN_MAIN);
  }*/
  void DGUSScreenHandler::SDCardRemoved() {
    //prnt_els2("SDCardRemoved");
    if (current_screen == DGUSLCD_SCREEN_SDFILELIST
        || (current_screen == DGUSLCD_SCREEN_CONFIRM && (ConfirmVP == VP_SD_AbortPrintConfirmed || ConfirmVP == VP_SD_FileSelectConfirm))
        || current_screen == DGUSLCD_SCREEN_SDPRINTMANIPULATION
    ) ScreenHandler.GotoScreen(DGUSLCD_SCREEN_MAIN);
  }

#endif // SDSUPPORT

/*void DGUSScreenHandler::ScreenChangeHook(DGUS_VP_Variable &var, void *val_ptr) {
  uint8_t *tmp = (uint8_t*)val_ptr;

  // The keycode in target is coded as <from-frame><to-frame>, so 0x0100A means
  // from screen 1 (main) to 10 (temperature). DGUSLCD_SCREEN_POPUP is special,
  // meaning "return to previous screen"
  DGUSLCD_Screens target = (DGUSLCD_Screens)tmp[1];

  DEBUG_ECHOLNPGM("\n DEBUG target", target);

  if (target == DGUSLCD_SCREEN_POPUP) {
    // Special handling for popup is to return to previous menu
    if (current_screen == DGUSLCD_SCREEN_POPUP && confirm_action_cb) confirm_action_cb();
    PopToOldScreen();
    return;
  }

  UpdateNewScreen(target);

  #ifdef DEBUG_DGUSLCD
    if (!DGUSLCD_FindScreenVPMapList(target)) DEBUG_ECHOLNPGM("WARNING: No screen Mapping found for ", target);
  #endif
}*/
void DGUSScreenHandler::ScreenChangeHook(DGUS_VP_Variable &var, void *val_ptr) {
  uint8_t *tmp = (uint8_t*)val_ptr;

  // The keycode in target is coded as <from-frame><to-frame>, so 0x0100A means
  // from screen 1 (main) to 10 (temperature). DGUSLCD_SCREEN_POPUP is special,
  // meaning "return to previous screen"
  DGUSLCD_Screens target = (DGUSLCD_Screens)tmp[1];
  on_file_list_screen = false; // reset

  if (target == DGUSLCD_SCREEN_POPUP) {
    // special handling for popup is to return to previous menu
    if (current_screen == DGUSLCD_SCREEN_POPUP && confirm_action_cb) confirm_action_cb();
    PopToOldScreen();
    return;
  }

  UpdateNewScreen(target);

  #ifdef DEBUG_DGUSLCD
    if (!DGUSLCD_FindScreenVPMapList(target)) DEBUG_ECHOLNPAIR("WARNING: No screen Mapping found for ", target);
  #endif
}

/*void DGUSScreenHandler::HandleManualMove(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleManualMove");

  int16_t movevalue = swap16(*(uint16_t*)val_ptr);
  #if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
    if (movevalue) {
      const uint16_t choice = *(uint16_t*)var.memadr;
      movevalue = movevalue < 0 ? -choice : choice;
    }
  #endif
  char axiscode;
  unsigned int speed = 1500; // FIXME: get default feedrate for manual moves, don't hardcode.

  switch (var.VP) {
    default: return;

    case VP_MOVE_X:
      axiscode = 'X';
      if (!ExtUI::canMove(ExtUI::axis_t::X)) goto cannotmove;
      break;

    case VP_MOVE_Y:
      axiscode = 'Y';
      if (!ExtUI::canMove(ExtUI::axis_t::Y)) goto cannotmove;
      break;

    case VP_MOVE_Z:
      axiscode = 'Z';
      speed = 300; // default to 5mm/s
      if (!ExtUI::canMove(ExtUI::axis_t::Z)) goto cannotmove;
      break;

    case VP_HOME_ALL: // only used for homing
      axiscode  = '\0';
      movevalue = 0; // ignore value sent from display, this VP is _ONLY_ for homing.
      break;
  }

  if (!movevalue) {
    // homing
    DEBUG_ECHOPGM(" homing ", AS_CHAR(axiscode));
    char buf[6] = "G28 X";
    buf[4] = axiscode;
    //DEBUG_ECHOPGM(" ", buf);
    queue.enqueue_one_now(buf);
    //DEBUG_ECHOLNPGM(" ✓");
    ForceCompleteUpdate();
    return;
  }
  else {
    // movement
    DEBUG_ECHOPGM(" move ", AS_CHAR(axiscode));
    bool old_relative_mode = relative_mode;
    if (!relative_mode) {
      //DEBUG_ECHOPGM(" G91");
      queue.enqueue_now(F("G91"));
      //DEBUG_ECHOPGM(" ✓ ");
    }
    char buf[32]; // G1 X9999.99 F12345
    unsigned int backup_speed = MMS_TO_MMM(feedrate_mm_s);
    char sign[] = "\0";
    int16_t value = movevalue / 100;
    if (movevalue < 0) { value = -value; sign[0] = '-'; }
    int16_t fraction = ABS(movevalue) % 100;
    snprintf_P(buf, 32, PSTR("G0 %c%s%d.%02d F%d"), axiscode, sign, value, fraction, speed);
    //DEBUG_ECHOPGM(" ", buf);
    queue.enqueue_one_now(buf);
    //DEBUG_ECHOLNPGM(" ✓ ");
    if (backup_speed != speed) {
      snprintf_P(buf, 32, PSTR("G0 F%d"), backup_speed);
      queue.enqueue_one_now(buf);
      //DEBUG_ECHOPGM(" ", buf);
    }
    // while (!enqueue_and_echo_command(buf)) idle();
    //DEBUG_ECHOLNPGM(" ✓ ");
    if (!old_relative_mode) {
      //DEBUG_ECHOPGM("G90");
      queue.enqueue_now(F("G90"));
      //DEBUG_ECHOPGM(" ✓ ");
    }
  }

  ForceCompleteUpdate();
  DEBUG_ECHOLNPGM("manmv done.");
  return;

  cannotmove:
    DEBUG_ECHOLNPGM(" cannot move ", AS_CHAR(axiscode));
    return;
}*/
void DGUSScreenHandler::HandleManualMove(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleManualMove");

  int16_t movevalue = swap16(*(uint16_t*)val_ptr);
  #if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
    if (movevalue) {
      const uint16_t choice = *(uint16_t*)var.memadr;
      movevalue = movevalue < 0 ? -choice : choice;
    }
  #endif
  char axiscode;
  unsigned int speed = 1500;  //FIXME: get default feedrate for manual moves, dont hardcode.

  switch (var.VP) {
    default: return;

    case VP_MOVE_X:
      axiscode = 'X';
      if (!ExtUI::canMove(ExtUI::axis_t::X)) goto cannotmove;
      break;

    case VP_MOVE_Y:
      axiscode = 'Y';
      if (!ExtUI::canMove(ExtUI::axis_t::Y)) goto cannotmove;
      break;

    case VP_MOVE_Z:
      axiscode = 'Z';
      speed = 300; // default to 5mm/s
      if (!ExtUI::canMove(ExtUI::axis_t::Z)) goto cannotmove;
      break;

    case VP_HOME_ALL: // only used for homing
      axiscode = '\0';
      movevalue = 0; // ignore value sent from display, this VP is _ONLY_ for homing.
      break;
  }

  if (!movevalue) {
    // homing
    //DEBUG_ECHOPAIR(" homing ", axiscode);
    DEBUG_ECHOPGM(" homing ", AS_CHAR(axiscode));
    char buf[6] = "G28 X";
    buf[4] = axiscode;
    //DEBUG_ECHOPAIR(" ", buf);
    queue.enqueue_one_now(buf);
    //DEBUG_ECHOLNPGM(" ✓");
    ScreenHandler.ForceCompleteUpdate();
    return;
  }
  else {
    //movement
    //DEBUG_ECHOPAIR(" move ", axiscode);
    DEBUG_ECHOPGM(" move ", AS_CHAR(axiscode));
    bool old_relative_mode = relative_mode;
    if (!relative_mode) {
      //DEBUG_ECHOPGM(" G91");
      queue.enqueue_now_P(PSTR("G91"));
      //DEBUG_ECHOPGM(" ✓ ");
    }
    char buf[32];  // G1 X9999.99 F12345
    unsigned int backup_speed = MMS_TO_MMM(feedrate_mm_s);
    char sign[]="\0";
    int16_t value = movevalue / 100;
    if (movevalue < 0) { value = -value; sign[0] = '-'; }
    int16_t fraction = ABS(movevalue) % 100;
    snprintf_P(buf, 32, PSTR("G0 %c%s%d.%02d F%d"), axiscode, sign, value, fraction, speed);
    //DEBUG_ECHOPAIR(" ", buf);
    queue.enqueue_one_now(buf);
    //DEBUG_ECHOLNPGM(" ✓ ");
    if (backup_speed != speed) {
      snprintf_P(buf, 32, PSTR("G0 F%d"), backup_speed);
      queue.enqueue_one_now(buf);
      //DEBUG_ECHOPAIR(" ", buf);
    }
    //while (!enqueue_and_echo_command(buf)) idle();
    //DEBUG_ECHOLNPGM(" ✓ ");
    if (!old_relative_mode) {
      //DEBUG_ECHOPGM("G90");
      queue.enqueue_now_P(PSTR("G90"));
      //DEBUG_ECHOPGM(" ✓ ");
    }
  }

  ScreenHandler.ForceCompleteUpdate();
  DEBUG_ECHOLNPGM("manmv done.");
  return;

  cannotmove:
  //DEBUG_ECHOLNPAIR(" cannot move ", axiscode);
  DEBUG_ECHOLNPGM(" cannot move ", AS_CHAR(axiscode));
  return;
}

#if HAS_PID_HEATING
  /*void DGUSScreenHandler::HandleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t rawvalue = swap16(*(uint16_t*)val_ptr);
    DEBUG_ECHOLNPGM("V1:", rawvalue);
    float value = (float)rawvalue / 10;
    DEBUG_ECHOLNPGM("V2:", value);
    float newvalue = 0;

    switch (var.VP) {
      default: return;
        #if HAS_HOTEND
          case VP_E0_PID_P: newvalue = value; break;
          case VP_E0_PID_I: newvalue = scalePID_i(value); break;
          case VP_E0_PID_D: newvalue = scalePID_d(value); break;
        #endif
        #if HAS_MULTI_HOTEND
          case VP_E1_PID_P: newvalue = value; break;
          case VP_E1_PID_I: newvalue = scalePID_i(value); break;
          case VP_E1_PID_D: newvalue = scalePID_d(value); break;
        #endif
        #if HAS_HEATED_BED
          case VP_BED_PID_P: newvalue = value; break;
          case VP_BED_PID_I: newvalue = scalePID_i(value); break;
          case VP_BED_PID_D: newvalue = scalePID_d(value); break;
        #endif
    }

    DEBUG_ECHOLNPGM("V3:", newvalue);
    *(float *)var.memadr = newvalue;

    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }*/
  void DGUSScreenHandler::HandleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t rawvalue = swap16(*(uint16_t*)val_ptr);
    //DEBUG_ECHOLNPAIR("V1:", rawvalue);
    DEBUG_ECHOLNPGM("V1:", rawvalue);
    float value = (float)rawvalue / 10;
    //DEBUG_ECHOLNPAIR("V2:", value);
    DEBUG_ECHOLNPGM("V2:", value);
    float newvalue = 0;

    switch (var.VP) {
      default: return;
      #if HOTENDS >= 1
        case VP_E0_PID_P: newvalue = value; break;
        case VP_E0_PID_I: newvalue = scalePID_i(value); break;
        case VP_E0_PID_D: newvalue = scalePID_d(value); break;
      #endif
      #if HOTENDS >= 2
        case VP_E1_PID_P: newvalue = value; break;
        case VP_E1_PID_I: newvalue = scalePID_i(value); break;
        case VP_E1_PID_D: newvalue = scalePID_d(value); break;
      #endif
      #if HAS_HEATED_BED
        case VP_BED_PID_P: newvalue = value; break;
        case VP_BED_PID_I: newvalue = scalePID_i(value); break;
        case VP_BED_PID_D: newvalue = scalePID_d(value); break;
      #endif
    }

    DEBUG_ECHOLNPAIR_F("V3:", newvalue);
    *(float *)var.memadr = newvalue;
    ScreenHandler.skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }
#endif // HAS_PID_HEATING

#if ENABLED(BABYSTEPPING)
  void DGUSScreenHandler::HandleLiveAdjustZ(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleLiveAdjustZ");

    int16_t flag = swap16(*(uint16_t*)val_ptr);
    int16_t steps = flag ? -20 : 20;
    ExtUI::smartAdjustAxis_steps(steps, ExtUI::axis_t::Z, true);
    ScreenHandler.ForceCompleteUpdate();
    (void)settings.save();
    return;
  }
#endif

bool DGUSScreenHandler::loop() {
  dgusdisplay.loop();

  const millis_t ms = millis();
  static millis_t next_event_ms = 0;

  if (!IsScreenComplete() || ELAPSED(ms, next_event_ms)) {
    next_event_ms = ms + DGUS_UPDATE_INTERVAL_MS;
    UpdateScreenVPData();
  }

  #if ENABLED(SHOW_BOOTSCREEN)
    static bool booted = false;

    if (!booted && TERN0(POWER_LOSS_RECOVERY, recovery.valid()))
      booted = true;

    if (!booted && ELAPSED(ms, BOOTSCREEN_TIMEOUT)) {
      booted = true;
      GotoScreen(TERN0(POWER_LOSS_RECOVERY, recovery.valid()) ? DGUSLCD_SCREEN_POWER_LOSS : DGUSLCD_SCREEN_MAIN);
    }
  #endif

  return IsScreenComplete();
}

#endif // DGUS_LCD_UI_FYSETC