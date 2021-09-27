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

#if HAS_DGUS_LCD

#include "DGUSScreenHandler.h"
#include "DGUSDisplay.h"
#include "DGUSVPVariable.h"
#include "DGUSDisplayDef.h"

#include "../../ui_api.h"
#include "../../../../MarlinCore.h"
#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../gcode/queue.h"
#include "../../../../module/planner.h"
#include "../../../../sd/cardreader.h"
#include "../../../../libs/duration_t.h"
#include "../../../../module/printcounter.h"

#include "../../module/settings.h"  //Elsan
#include "../../../gcode/gcode.h"
void usb_ls2(void);
void prnt_els2(char * str);
extern char buf_main[50][200];
int file_cnt3=0;
extern int file_cnt2, file_cnt;
extern int print_percent;
extern GcodeSuite gcode;  //Elsan

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../../feature/powerloss.h"
#endif

uint16_t DGUSScreenHandler::ConfirmVP;

#if ENABLED(SDSUPPORT)
  int16_t DGUSScreenHandler::top_file = 0;
  int16_t DGUSScreenHandler::file_to_print = 0;
  static ExtUI::FileList filelist;
#endif

void (*DGUSScreenHandler::confirm_action_cb)() = nullptr;

//DGUSScreenHandler ScreenHandler;

DGUSLCD_Screens DGUSScreenHandler::current_screen;
DGUSLCD_Screens DGUSScreenHandler::past_screens[NUM_PAST_SCREENS];
uint8_t DGUSScreenHandler::update_ptr;
uint16_t DGUSScreenHandler::skipVP;
bool DGUSScreenHandler::ScreenComplete;

//DGUSDisplay dgusdisplay;

// endianness swap
uint16_t swap16(const uint16_t value) { return (value & 0xffU) << 8U | (value >> 8U); }

void DGUSScreenHandler::sendinfoscreen(const char* line1, const char* line2, const char* line3, const char* line4, bool l1inflash, bool l2inflash, bool l3inflash, bool l4inflash) {
  DGUS_VP_Variable ramcopy;
  if (populate_VPVar(VP_MSGSTR1, &ramcopy)) {
    ramcopy.memadr = (void*) line1;
    l1inflash ? DGUSScreenHandler::DGUSLCD_SendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::DGUSLCD_SendStringToDisplay(ramcopy);
  }
  if (populate_VPVar(VP_MSGSTR2, &ramcopy)) {
    ramcopy.memadr = (void*) line2;
    l2inflash ? DGUSScreenHandler::DGUSLCD_SendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::DGUSLCD_SendStringToDisplay(ramcopy);
  }
  if (populate_VPVar(VP_MSGSTR3, &ramcopy)) {
    ramcopy.memadr = (void*) line3;
    l3inflash ? DGUSScreenHandler::DGUSLCD_SendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::DGUSLCD_SendStringToDisplay(ramcopy);
  }
  if (populate_VPVar(VP_MSGSTR4, &ramcopy)) {
    ramcopy.memadr = (void*) line4;
    l4inflash ? DGUSScreenHandler::DGUSLCD_SendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::DGUSLCD_SendStringToDisplay(ramcopy);
  }
}

void DGUSScreenHandler::HandleUserConfirmationPopUp(uint16_t VP, const char* line1, const char* line2, const char* line3, const char* line4, bool l1, bool l2, bool l3, bool l4) {
  //SERIAL_ECHOLNPGM("");SERIAL_ECHOLNPGM("HandleUserConfirmationPopUp");
  if (current_screen == DGUSLCD_SCREEN_CONFIRM) {
    // Already showing a pop up, so we need to cancel that first.
    PopToOldScreen();
  }

  ConfirmVP = VP;
  sendinfoscreen(line1, line2, line3, line4, l1, l2, l3, l4);
  ScreenHandler.GotoScreen(DGUSLCD_SCREEN_CONFIRM);
}

void DGUSScreenHandler::setstatusmessage(const char *msg) {
  DGUS_VP_Variable ramcopy;
  if (populate_VPVar(VP_M117, &ramcopy)) {
    ramcopy.memadr = (void*) msg;
    DGUSLCD_SendStringToDisplay(ramcopy);
  }
}

void DGUSScreenHandler::setstatusmessagePGM(PGM_P const msg) {
  DGUS_VP_Variable ramcopy;
  if (populate_VPVar(VP_M117, &ramcopy)) {
    ramcopy.memadr = (void*) msg;
    DGUSLCD_SendStringToDisplayPGM(ramcopy);
  }
}

// Send an 8 bit or 16 bit value to the display.
void DGUSScreenHandler::DGUSLCD_SendWordValueToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    //DEBUG_ECHOPAIR(" DGUS_LCD_SendWordValueToDisplay ", var.VP);
    //DEBUG_ECHOLNPAIR(" data ", *(uint16_t *)var.memadr);
    if (var.size > 1)
      dgusdisplay.WriteVariable(var.VP, *(int16_t*)var.memadr);
    else
      dgusdisplay.WriteVariable(var.VP, *(int8_t*)var.memadr);
  }
}

// Send an uint8_t between 0 and 255 to the display, but scale to a percentage (0..100)
void DGUSScreenHandler::DGUSLCD_SendPercentageToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    //DEBUG_ECHOPAIR(" DGUS_LCD_SendWordValueToDisplay ", var.VP);
    //DEBUG_ECHOLNPAIR(" data ", *(uint16_t *)var.memadr);
    uint16_t tmp = *(uint8_t *) var.memadr +1 ; // +1 -> avoid rounding issues for the display.
    tmp = map(tmp, 0, 255, 0, 100);
    dgusdisplay.WriteVariable(var.VP, tmp);
  }
}

// Send the current print progress to the display.
void DGUSScreenHandler::DGUSLCD_SendPrintProgressToDisplay(DGUS_VP_Variable &var) {
  //DEBUG_ECHOPAIR(" DGUSLCD_SendPrintProgressToDisplay ", var.VP);
  uint16_t tmp = ExtUI::getProgress_percent();
  //DEBUG_ECHOLNPAIR(" data ", tmp);
  //tmp=card.percentDone(); //Elsan could also be used.
  print_percent=tmp;
  dgusdisplay.WriteVariable(var.VP, tmp);
}

// Send the current print progress to the display.
void DGUSScreenHandler::DGUSLCD_SendPrintProgressBarToDisplay(DGUS_VP_Variable &var) {
  // don't go to 100 since we don't have that much on dgus
  uint16_t tmp = _MIN(99, ExtUI::getProgress_percent());
  tmp = int(tmp / 10);
  uint16_t data_to_send = swap16(tmp);
  dgusdisplay.WriteVariable(var.VP, /*data_to_send*/tmp);
}

// Send the current print time to the display.
// It is using a hex display for that: It expects BSD coded data in the format xxyyzz
void DGUSScreenHandler::DGUSLCD_SendPrintTimeToDisplay(DGUS_VP_Variable &var) {
  duration_t elapsed = print_job_timer.duration();
  char buf[32];
  elapsed.toString(buf);
  dgusdisplay.WriteVariable(VP_PrintTime, buf, var.size, true);
}

// Send an uint8_t between 0 and 100 to a variable scale to 0..255
void DGUSScreenHandler::DGUSLCD_PercentageToUint8(DGUS_VP_Variable &var, void *val_ptr) {
  if (var.memadr) {
    uint16_t value = swap16(*(uint16_t*)val_ptr);
    *(uint8_t*)var.memadr = map(constrain(value, 0, 100), 0, 100, 0, 255);
  }
}

// Sends a (RAM located) string to the DGUS Display
// (Note: The DGUS Display does not clear after the \0, you have to
// overwrite the remainings with spaces.// var.size has the display buffer size!
void DGUSScreenHandler::DGUSLCD_SendStringToDisplay(DGUS_VP_Variable &var) {
  char *tmp = (char*) var.memadr;
  dgusdisplay.WriteVariable(var.VP, tmp, var.size, true);
}

// Sends a (flash located) string to the DGUS Display
// (Note: The DGUS Display does not clear after the \0, you have to
// overwrite the remainings with spaces.// var.size has the display buffer size!
void DGUSScreenHandler::DGUSLCD_SendStringToDisplayPGM(DGUS_VP_Variable &var) {
  char *tmp = (char*) var.memadr;
  dgusdisplay.WriteVariablePGM(var.VP, tmp, var.size, true);
}

#if HAS_PID_HEATING
  void DGUSScreenHandler::DGUSLCD_SendTemperaturePID(DGUS_VP_Variable &var) {
    float value = *(float *)var.memadr;
    float valuesend = 0;
    switch (var.VP) {
      default: return;
      #if HOTENDS >= 1
        case VP_E0_PID_P: valuesend = value; break;
        case VP_E0_PID_I: valuesend = unscalePID_i(value); break;
        case VP_E0_PID_D: valuesend = unscalePID_d(value); break;
      #endif
      #if HOTENDS >= 2
        case VP_E1_PID_P: valuesend = value; break;
        case VP_E1_PID_I: valuesend = unscalePID_i(value); break;
        case VP_E1_PID_D: valuesend = unscalePID_d(value); break;
      #endif
      #if HAS_HEATED_BED
        case VP_BED_PID_P: valuesend = value; break;
        case VP_BED_PID_I: valuesend = unscalePID_i(value); break;
        case VP_BED_PID_D: valuesend = unscalePID_d(value); break;
      #endif
    }

    valuesend *= cpow(10, 1);
    union { int16_t i; char lb[2]; } endian;

    char tmp[2];
    endian.i = valuesend;
    tmp[0] = endian.lb[1];
    tmp[1] = endian.lb[0];
    dgusdisplay.WriteVariable(var.VP, tmp, 2);
  }
#endif

#if ENABLED(PRINTCOUNTER)

  // Send the accumulate print time to the display.
  // It is using a hex display for that: It expects BSD coded data in the format xxyyzz
  void DGUSScreenHandler::DGUSLCD_SendPrintAccTimeToDisplay(DGUS_VP_Variable &var) {
    printStatistics state = print_job_timer.getStats();
    char buf[21];
    duration_t elapsed = state.printTime;
    elapsed.toString(buf);
    dgusdisplay.WriteVariable(VP_PrintAccTime, buf, var.size, true);
  }

  void DGUSScreenHandler::DGUSLCD_SendPrintsTotalToDisplay(DGUS_VP_Variable &var) {
    printStatistics state = print_job_timer.getStats();
    char buf[21];
    sprintf_P(buf, PSTR("%u"), state.totalPrints);
    dgusdisplay.WriteVariable(VP_PrintsTotal, buf, var.size, true);
  }

#endif

// Send fan status value to the display.
#if HAS_FAN
  void DGUSScreenHandler::DGUSLCD_SendFanStatusToDisplay(DGUS_VP_Variable &var) {
    if (var.memadr) {
      DEBUG_ECHOPAIR(" DGUSLCD_SendFanStatusToDisplay ", var.VP);
      DEBUG_ECHOLNPAIR(" data ", *(uint8_t *)var.memadr);
      uint16_t data_to_send = 0;
      if (*(uint8_t *) var.memadr) data_to_send = 1;
      dgusdisplay.WriteVariable(var.VP, data_to_send);
    }
  }
#endif

// Send heater status value to the display.
void DGUSScreenHandler::DGUSLCD_SendHeaterStatusToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    DEBUG_ECHOPAIR(" DGUSLCD_SendHeaterStatusToDisplay ", var.VP);
    DEBUG_ECHOLNPAIR(" data ", *(int16_t *)var.memadr);
    uint16_t data_to_send = 0;
    if (*(int16_t *) var.memadr) data_to_send = 1;
    dgusdisplay.WriteVariable(var.VP, data_to_send);
  }
}

#if ENABLED(DGUS_UI_WAITING)
  void DGUSScreenHandler::DGUSLCD_SendWaitingStatusToDisplay(DGUS_VP_Variable &var) {
    // In FYSETC UI design there are 10 statuses to loop
    static uint16_t period = 0;
    static uint16_t index = 0;
    //DEBUG_ECHOPAIR(" DGUSLCD_SendWaitingStatusToDisplay ", var.VP);
    //DEBUG_ECHOLNPAIR(" data ", swap16(index));
    if (period++ > DGUS_UI_WAITING_STATUS_PERIOD) {
      dgusdisplay.WriteVariable(var.VP, index);
      //DEBUG_ECHOLNPAIR(" data ", swap16(index));
      if (++index >= DGUS_UI_WAITING_STATUS) index = 0;
      period = 0;
    }
  }
#endif

#if ENABLED(SDSUPPORT)

  void DGUSScreenHandler::ScreenChangeHookIfSD(DGUS_VP_Variable &var, void *val_ptr) {
    //prnt_els2("ScreenChangeHookIfSD");
    // default action executed when there is a SD card, but not printing
    if (ExtUI::isMediaInserted() && !ExtUI::isPrintingFromMedia()) {
      ScreenChangeHook(var, val_ptr);
      dgusdisplay.RequestScreen(current_screen);
      return;
    }

    // if we are printing, we jump to two screens after the requested one.
    // This should host e.g a print pause / print abort / print resume dialog.
    // This concept allows to recycle this hook for other file
    if (ExtUI::isPrintingFromMedia() && !card.flag.abort_sd_printing) {
      GotoScreen(DGUSLCD_SCREEN_SDPRINTMANIPULATION);
      return;
    }

    // Don't let the user in the dark why there is no reaction.
    if (!ExtUI::isMediaInserted()) {
      setstatusmessagePGM(GET_TEXT(MSG_NO_MEDIA));
      return;
    }
    if (card.flag.abort_sd_printing) {
      setstatusmessagePGM(GET_TEXT(MSG_MEDIA_ABORTING));
      return;
    }
  }

  //Elsan
  void /*DGUSScreenVariableHandler*/DGUSScreenHandler::DGUSLCD_SD_RefreshFilelist(DGUS_VP_Variable& var, void *val_ptr) {
     //filelist./*reload*/refresh();
     //prnt_els2("DGUSLCD_SD_RefreshFilelist");
     file_cnt2=0; //File counter.
     file_cnt=0; //File counter.
     file_cnt3=0; 
     top_file = 0;  
     file_to_print=0; 
     
     for(int a=0;a<50;a++) memset(&buf_main[a][0],0x00,200)/*buf_main[a][0]=0*/;
     usb_ls2(); 
     
     //char buf[21];
     //sprintf(buf, "%s", "ABC");
     //dgusdisplay.WriteVariable(VP_SD_Refresh_List, buf, var.size, true);
     dgusdisplay.WriteVariable(VP_SD_FileName0, &buf_main[0][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName1, &buf_main[1][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName2, &buf_main[2][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName3, &buf_main[3][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName4, &buf_main[4][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName5, &buf_main[5][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName6, &buf_main[6][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName7, &buf_main[7][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName8, &buf_main[8][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName9, &buf_main[9][0], 40, true);
     //ForceCompleteUpdate(); //test
  }
  ////////////////////////////

  void DGUSScreenHandler::DGUSLCD_SD_ScrollFilelist(DGUS_VP_Variable& var, void *val_ptr) {
    //prnt_els2("DGUSLCD_SD_ScrollFilelist");
    auto old_top = top_file;
    const int16_t scroll = (int16_t)swap16(*(uint16_t*)val_ptr);
    if (scroll) {
      top_file += scroll;
      DEBUG_ECHOPAIR("new topfile calculated:", top_file);
      if (top_file < 0) {
        top_file = 0;
        DEBUG_ECHOLNPGM("Top of filelist reached");
      }
      else {
        int16_t max_top = filelist.count()/*file_cnt2*/ - DGUS_SD_FILESPERSCREEN;
        NOLESS(max_top, 0);
        NOMORE(top_file, max_top);
      }
      DEBUG_ECHOPAIR("new topfile adjusted:", top_file);
    }
    else if (!filelist.isAtRootDir()) {
      //filelist.upDir();
      top_file = 0;
      ForceCompleteUpdate();
    }
    
    if (old_top != top_file) ForceCompleteUpdate();
      
     file_cnt3 += scroll;
     if(file_cnt3 >= file_cnt2) file_cnt3 -= scroll;
     if(file_cnt3 < 0) file_cnt3 += 5;
     dgusdisplay.WriteVariable(VP_SD_FileName0, &buf_main[file_cnt3+0][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName1, &buf_main[file_cnt3+1][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName2, &buf_main[file_cnt3+2][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName3, &buf_main[file_cnt3+3][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName4, &buf_main[file_cnt3+4][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName5, &buf_main[file_cnt3+5][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName6, &buf_main[file_cnt3+6][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName7, &buf_main[file_cnt3+7][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName8, &buf_main[file_cnt3+8][0], 40, true);
     dgusdisplay.WriteVariable(VP_SD_FileName9, &buf_main[file_cnt3+9][0], 40, true);
     //ForceCompleteUpdate(); //test
  }

  void DGUSScreenHandler::DGUSLCD_SD_FileSelected(DGUS_VP_Variable &var, void *val_ptr) {
    //prnt_els2("DGUSLCD_SD_FileSelected");
    uint16_t touched_nr = (int16_t)swap16(*(uint16_t*)val_ptr) + top_file;
    //Elsan dis temporarily
    //usb_ls2();
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

  void DGUSScreenHandler::DGUSLCD_SD_StartPrint(DGUS_VP_Variable &var, void *val_ptr) {
    //Elsan from old version
    uint16_t value = swap16(*(uint16_t*)val_ptr);
    if (!value) {
      PopToOldScreen();
      return;
    }
    /////////////////////////////////////

    //Elsan dis temporarily
    //if (!filelist.seek(file_to_print)) return;

    ExtUI::printFile(/*filelist.shortFilename()*/buf_main[file_to_print+file_cnt3]);
    ScreenHandler.GotoScreen(
      #if ENABLED(DGUS_LCD_UI_ORIGIN)
        DGUSLCD_SCREEN_STATUS
      #else
        DGUSLCD_SCREEN_SDPRINTMANIPULATION
      #endif
    );
  }

  void DGUSScreenHandler::DGUSLCD_SD_ResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr) {
    //prnt_els2("DGUSLCD_SD_ResumePauseAbort");
    //if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.
    switch (swap16(*(uint16_t*)val_ptr)) {
      case 0:  // Resume
        //SERIAL_ECHOLNPGM("DGUSLCD_SD_Resume");
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

  void DGUSScreenHandler::DGUSLCD_SD_ReallyAbort(DGUS_VP_Variable &var, void *val_ptr) {
    //Elsan from old version
    //SERIAL_ECHOLNPGM("DGUSLCD_SD_ReallyAbort");
    uint16_t value = swap16(*(uint16_t*)val_ptr);
    if (!value) {
      PopToOldScreen();
      return;
    } 
    ////////////////////////
    ExtUI::stopPrint(); //Elsan dis. Similar to abortSDPrinting triggered after M524. Can be used.
    //queue.enqueue_now_P("M524");  //Elsan add
    GotoScreen(DGUSLCD_SCREEN_MAIN);
  }

  void DGUSScreenHandler::DGUSLCD_SD_PrintTune(DGUS_VP_Variable &var, void *val_ptr) {
    //prnt_els2("DGUSLCD_SD_PrintTune");
    if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.
    GotoScreen(DGUSLCD_SCREEN_SDPRINTTUNE);
  }

  void DGUSScreenHandler::DGUSLCD_SD_SendFilename(DGUS_VP_Variable& var) {
    //prnt_els2("DGUSLCD_SD_SendFilename");
    uint16_t target_line = (var.VP - VP_SD_FileName0) / VP_SD_FileName_LEN;
    if (target_line > DGUS_SD_FILESPERSCREEN) return;
    char tmpfilename[VP_SD_FileName_LEN + 1] = "";
    var.memadr = (void*)tmpfilename;
    if (filelist.seek(top_file + target_line))
      snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s%c"), filelist.filename(), filelist.isDir() ? '/' : 0);
    DGUSLCD_SendStringToDisplay(var);
  }

  void DGUSScreenHandler::SDCardInserted() {
    //prnt_els2("SDCardInserted");
    top_file = 0;
    filelist.refresh(); 
    auto cs = ScreenHandler.getCurrentScreen();
    if (cs == DGUSLCD_SCREEN_MAIN || cs == DGUSLCD_SCREEN_STATUS)
      ScreenHandler.GotoScreen(DGUSLCD_SCREEN_SDFILELIST);
  }

  void DGUSScreenHandler::SDCardRemoved() {
    //prnt_els2("SDCardRemoved");
    if (current_screen == DGUSLCD_SCREEN_SDFILELIST
        || (current_screen == DGUSLCD_SCREEN_CONFIRM && (ConfirmVP == VP_SD_AbortPrintConfirmed || ConfirmVP == VP_SD_FileSelectConfirm))
        || current_screen == DGUSLCD_SCREEN_SDPRINTMANIPULATION
    ) ScreenHandler.GotoScreen(DGUSLCD_SCREEN_MAIN);
  }

  void DGUSScreenHandler::SDCardError() {
    //prnt_els2("SDCardError");
    DGUSScreenHandler::SDCardRemoved();
    ScreenHandler.sendinfoscreen(PSTR("NOTICE"), nullptr, PSTR("SD card error"), nullptr, true, true, true, true);
    ScreenHandler.SetupConfirmAction(nullptr);
    ScreenHandler.GotoScreen(DGUSLCD_SCREEN_POPUP);
  }

#endif // SDSUPPORT

void DGUSScreenHandler::ScreenConfirmedOK(DGUS_VP_Variable &var, void *val_ptr) {
  DGUS_VP_Variable ramcopy;
  if (!populate_VPVar(ConfirmVP, &ramcopy)) return;
  if (ramcopy.set_by_display_handler) ramcopy.set_by_display_handler(ramcopy, val_ptr);
}

const uint16_t* DGUSLCD_FindScreenVPMapList(uint8_t screen) {
  const uint16_t *ret;
  const struct VPMapping *map = VPMap;
  while (ret = (uint16_t*) pgm_read_ptr(&(map->VPList))) {
    if (pgm_read_byte(&(map->screen)) == screen) return ret;
    map++;
  }
  return nullptr;
}

const DGUS_VP_Variable* DGUSLCD_FindVPVar(const uint16_t vp) {
  const DGUS_VP_Variable *ret = ListOfVP;
  do {
    const uint16_t vpcheck = pgm_read_word(&(ret->VP));
    if (vpcheck == 0) break;
    if (vpcheck == vp) return ret;
    ++ret;
  } while (1);

  DEBUG_ECHOLNPAIR("FindVPVar NOT FOUND ", vp);
  return nullptr;
}

void DGUSScreenHandler::ScreenChangeHookIfIdle(DGUS_VP_Variable &var, void *val_ptr) {
  if (!ExtUI::isPrinting()) {
    ScreenChangeHook(var, val_ptr);
    dgusdisplay.RequestScreen(current_screen);
  }
}

void DGUSScreenHandler::ScreenChangeHook(DGUS_VP_Variable &var, void *val_ptr) {
  uint8_t *tmp = (uint8_t*)val_ptr;

  // The keycode in target is coded as <from-frame><to-frame>, so 0x0100A means
  // from screen 1 (main) to 10 (temperature). DGUSLCD_SCREEN_POPUP is special,
  // meaning "return to previous screen"
  DGUSLCD_Screens target = (DGUSLCD_Screens)tmp[1];

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

void DGUSScreenHandler::HandleAllHeatersOff(DGUS_VP_Variable &var, void *val_ptr) {
  thermalManager.disable_all_heaters();
  ScreenHandler.ForceCompleteUpdate(); // hint to send all data.
}

void DGUSScreenHandler::HandleTemperatureChanged(DGUS_VP_Variable &var, void *val_ptr) {
  uint16_t newvalue = swap16(*(uint16_t*)val_ptr);
  uint16_t acceptedvalue;

  switch (var.VP) {
    default: return;
    #if HOTENDS >= 1
      case VP_T_E0_Set:
        thermalManager.setTargetHotend(newvalue, 0);
        acceptedvalue = thermalManager.temp_hotend[0].target;
        break;
    #endif
    #if HOTENDS >= 2
      case VP_T_E1_Set:
        thermalManager.setTargetHotend(newvalue, 1);
        acceptedvalue = thermalManager.temp_hotend[1].target;
      break;
    #endif
    #if HAS_HEATED_BED
      case VP_T_Bed_Set:
        thermalManager.setTargetBed(newvalue);
        acceptedvalue = thermalManager.temp_bed.target;
        break;
    #endif
  }

  // reply to display the new value to update the view if the new value was rejected by the Thermal Manager.
  if (newvalue != acceptedvalue && var.send_to_display_handler) var.send_to_display_handler(var);
  ScreenHandler.skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandler::HandleFlowRateChanged(DGUS_VP_Variable &var, void *val_ptr) {
  #if EXTRUDERS
    uint16_t newvalue = swap16(*(uint16_t*)val_ptr);
    uint8_t target_extruder;
    switch (var.VP) {
      default: return;
      #if HOTENDS >= 1
        case VP_Flowrate_E0: target_extruder = 0; break;
      #endif
      #if HOTENDS >= 2
        case VP_Flowrate_E1: target_extruder = 1; break;
      #endif
    }

    planner.set_flow(target_extruder, newvalue);
    ScreenHandler.skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  #else
    UNUSED(var); UNUSED(val_ptr);
  #endif
}

void DGUSScreenHandler::HandleManualExtrude(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleManualExtrude");

  int16_t movevalue = swap16(*(uint16_t*)val_ptr);
  float target = movevalue * 0.01f;
  ExtUI::extruder_t target_extruder;

  switch (var.VP) {
    #if HOTENDS >= 1
      case VP_MOVE_E0: target_extruder = ExtUI::extruder_t::E0; break;
    #endif
    #if HOTENDS >= 2
      case VP_MOVE_E1: target_extruder = ExtUI::extruder_t::E1; break;
    #endif
    default: return;
  }

  target += ExtUI::getAxisPosition_mm(target_extruder);
  ExtUI::setAxisPosition_mm(target, target_extruder);
  skipVP = var.VP;
}

#if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
  void DGUSScreenHandler::HandleManualMoveOption(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleManualMoveOption");
    *(uint16_t*)var.memadr = swap16(*(uint16_t*)val_ptr);
  }
#endif

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
    DEBUG_ECHOPAIR(" homing ", axiscode);
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
    DEBUG_ECHOPAIR(" move ", axiscode);
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
  DEBUG_ECHOLNPAIR(" cannot move ", axiscode);
  return;
}

void DGUSScreenHandler::HandleEepromSaveRestoreSettings(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleEepromSaveRestoreSettings");

  char buf[5];
  const int16_t val = swap16(*(uint16_t*)val_ptr);
  strcpy_P(buf, val ? PSTR("M500") : PSTR("M501"));

  queue.enqueue_one_now(buf);
}

void DGUSScreenHandler::HandleEndPreheat(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleEndPreheat");

  const int16_t val = swap16(*(uint16_t*)val_ptr);
  if (!val) return;
  #if HOTENDS >= 1
    thermalManager.setTargetHotend(0, 0);
    #if HAS_HEATED_BED
      thermalManager.setTargetBed(0);
    #endif
  #endif
}

void DGUSScreenHandler::HandleMotorLockUnlock(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleMotorLockUnlock");

  char buf[4];
  const int16_t lock = swap16(*(uint16_t*)val_ptr);
  strcpy_P(buf, lock ? PSTR("M18") : PSTR("M17"));

  //DEBUG_ECHOPAIR(" ", buf);
  queue.enqueue_one_now(buf);
}

#if ENABLED(POWER_LOSS_RECOVERY)

  void DGUSScreenHandler::HandlePowerLossRecovery(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t value = swap16(*(uint16_t*)val_ptr);
    if (value) {
      queue.inject_P(PSTR("M1000"));
      ScreenHandler.GotoScreen(DGUSLCD_SCREEN_SDPRINTMANIPULATION);
    }
    else {
      recovery.cancel();
      ScreenHandler.GotoScreen(DGUSLCD_SCREEN_STATUS);
    }
  }

#endif

void DGUSScreenHandler::HandleSettings(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleSettings");
  uint16_t value = swap16(*(uint16_t*)val_ptr);
  switch (value) {
    default: break;
    case 1:
      TERN_(PRINTCOUNTER, print_job_timer.initStats());
      queue.inject_P(PSTR("M502\nM500"));
      break;
    case 2: queue.inject_P(PSTR("M501")); break;
    case 3: queue.inject_P(PSTR("M500")); break;
  }
}

void DGUSScreenHandler::HandleStepPerMMChanged(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleStepPerMMChanged");

  uint16_t value_raw = swap16(*(uint16_t*)val_ptr);
  DEBUG_ECHOLNPAIR("value_raw:", value_raw);
  float value = (float)value_raw/10;
  ExtUI::axis_t axis;
  switch (var.VP) {
    case VP_X_STEP_PER_MM: axis = ExtUI::axis_t::X; break;
    case VP_Y_STEP_PER_MM: axis = ExtUI::axis_t::Y; break;
    case VP_Z_STEP_PER_MM: axis = ExtUI::axis_t::Z; break;
    default: return;
  }
  DEBUG_ECHOLNPAIR_F("value:", value);
  ExtUI::setAxisSteps_per_mm(value, axis);
  DEBUG_ECHOLNPAIR_F("value_set:", ExtUI::getAxisSteps_per_mm(axis));
  ScreenHandler.skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  return;
}

void DGUSScreenHandler::HandleStepPerMMExtruderChanged(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleStepPerMMExtruderChanged");

  uint16_t value_raw = swap16(*(uint16_t*)val_ptr);
  DEBUG_ECHOLNPAIR("value_raw:", value_raw);
  float value = (float)value_raw/10;
  ExtUI::extruder_t extruder;
  switch (var.VP) {
    default: return;
    #if HOTENDS >= 1
      case VP_E0_STEP_PER_MM: extruder = ExtUI::extruder_t::E0; break;
    #endif
    #if HOTENDS >= 2
      case VP_E1_STEP_PER_MM: extruder = ExtUI::extruder_t::E1; break;
    #endif
  }
  DEBUG_ECHOLNPAIR_F("value:", value);
  ExtUI::setAxisSteps_per_mm(value,extruder);
  DEBUG_ECHOLNPAIR_F("value_set:", ExtUI::getAxisSteps_per_mm(extruder));
  ScreenHandler.skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  return;
}

#if HAS_PID_HEATING
  void DGUSScreenHandler::HandleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t rawvalue = swap16(*(uint16_t*)val_ptr);
    DEBUG_ECHOLNPAIR("V1:", rawvalue);
    float value = (float)rawvalue / 10;
    DEBUG_ECHOLNPAIR("V2:", value);
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

  void DGUSScreenHandler::HandlePIDAutotune(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandlePIDAutotune");

    char buf[32] = {0};

    switch (var.VP) {
      default: break;
      #if ENABLED(PIDTEMP)
        #if HOTENDS >= 1
          case VP_PID_AUTOTUNE_E0: // Autotune Extruder 0
            sprintf(buf, "M303 E%d C5 S210 U1", ExtUI::extruder_t::E0);
            break;
        #endif
        #if HOTENDS >= 2
          case VP_PID_AUTOTUNE_E1:
            sprintf(buf, "M303 E%d C5 S210 U1", ExtUI::extruder_t::E1);
            break;
        #endif
      #endif
      #if ENABLED(PIDTEMPBED)
        case VP_PID_AUTOTUNE_BED:
          sprintf(buf, "M303 E-1 C5 S70 U1");
          break;
      #endif
    }

    if (buf[0]) queue.enqueue_one_now(buf);

    #if ENABLED(DGUS_UI_WAITING)
      sendinfoscreen(PSTR("PID is autotuning"), PSTR("please wait"), NUL_STR, NUL_STR, true, true, true, true);
      GotoScreen(DGUSLCD_SCREEN_WAITING);
    #endif
  }
#endif

#if HAS_BED_PROBE
  void DGUSScreenHandler::HandleProbeOffsetZChanged(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleProbeOffsetZChanged");

    const float offset = float(int16_t(swap16(*(uint16_t*)val_ptr))) / 100.0f;
    ExtUI::setZOffset_mm(offset);
    ScreenHandler.skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
    return;
  }
#endif

#if ENABLED(BABYSTEPPING)
  void DGUSScreenHandler::HandleLiveAdjustZ(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleLiveAdjustZ");

    int16_t flag = swap16(*(uint16_t*)val_ptr);
    int16_t steps = flag ? -20 : 20;
    ExtUI::smartAdjustAxis_steps(steps, ExtUI::axis_t::Z, true);
    ScreenHandler.ForceCompleteUpdate();
    
    return;
  }
#endif

#if HAS_FAN
  void DGUSScreenHandler::HandleFanControl(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleFanControl");
    *(uint8_t*)var.memadr = *(uint8_t*)var.memadr > 0 ? 0 : 255;
  }
#endif

void DGUSScreenHandler::HandleHeaterControl(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleHeaterControl");

  uint8_t preheat_temp = 0;
  switch (var.VP) {
    #if HOTENDS >= 1
      case VP_E0_CONTROL:
    #endif
    #if HOTENDS >= 2
      case VP_E1_CONTROL:
    #endif
    #if HOTENDS >= 3
      case VP_E2_CONTROL:
    #endif
      preheat_temp = PREHEAT_1_TEMP_HOTEND;
      break;

    case VP_BED_CONTROL:
      preheat_temp = PREHEAT_1_TEMP_BED;
      break;
  }

  *(int16_t*)var.memadr = *(int16_t*)var.memadr > 0 ? 0 : preheat_temp;
}

#if ENABLED(DGUS_PREHEAT_UI)

  void DGUSScreenHandler::HandlePreheat(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandlePreheat");

    uint8_t e_temp = 0;
    TERN_(HAS_HEATED_BED, uint8_t bed_temp = 0);
    const uint16_t preheat_option = swap16(*(uint16_t*)val_ptr);
    switch (preheat_option) {
      default:
      case 0: // Preheat PLA
        #if defined(PREHEAT_1_TEMP_HOTEND) && defined(PREHEAT_1_TEMP_BED)
          e_temp = PREHEAT_1_TEMP_HOTEND;
          TERN_(HAS_HEATED_BED, bed_temp = PREHEAT_1_TEMP_BED);
        #endif
        break;
      case 1: // Preheat ABS
        #if defined(PREHEAT_2_TEMP_HOTEND) && defined(PREHEAT_2_TEMP_BED)
          e_temp = PREHEAT_2_TEMP_HOTEND;
          TERN_(HAS_HEATED_BED, bed_temp = PREHEAT_2_TEMP_BED);
        #endif
        break;
      case 2: // Preheat PET
        #if defined(PREHEAT_3_TEMP_HOTEND) && defined(PREHEAT_3_TEMP_BED)
          e_temp = PREHEAT_3_TEMP_HOTEND;
          TERN_(HAS_HEATED_BED, bed_temp = PREHEAT_3_TEMP_BED);
        #endif
        break;
      case 3: // Preheat FLEX
        #if defined(PREHEAT_4_TEMP_HOTEND) && defined(PREHEAT_4_TEMP_BED)
          e_temp = PREHEAT_4_TEMP_HOTEND;
          TERN_(HAS_HEATED_BED, bed_temp = PREHEAT_4_TEMP_BED);
        #endif
        break;
      case 7: break; // Custom preheat
      case 9: break; // Cool down
    }

    switch (var.VP) {
      default: return;
      #if HOTENDS >= 1
        case VP_E0_BED_PREHEAT:
          thermalManager.setTargetHotend(e_temp, 0);
          TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(bed_temp));
          break;
      #endif
      #if HOTENDS >= 2
        case VP_E1_BED_PREHEAT:
          thermalManager.setTargetHotend(e_temp, 1);
          TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(bed_temp));
        break;
      #endif
    }

    // Go to the preheat screen to show the heating progress
    GotoScreen(DGUSLCD_SCREEN_PREHEAT);
  }

#endif

#if ENABLED(SKEW_CORRECTION)
  typedef struct  {
    float xy_diag_ac = XY_DIAG_AC;
    float xy_diag_bd = XY_DIAG_BD;
    float xy_side_ad = XY_SIDE_AD;
    float xy_skew_factor = 0.0;
  } xy_calib_data_t;
  static xy_calib_data_t xy_calib_data;

  void DGUSScreenHandler::HandleSkewVarsChanged(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleSkewVarsChanged");

    uint16_t value_raw = swap16(*(uint16_t*)val_ptr);
    float value = (float)value_raw / 100;
    DEBUG_ECHOLNPAIR("value_raw:", value_raw);
    switch (var.VP) {
      case VP_XY_DIAG_AC: xy_calib_data.xy_diag_ac = value; break;
      case VP_XY_DIAG_BD: xy_calib_data.xy_diag_bd = value; break;
      case VP_XY_SIDE_AD: xy_calib_data.xy_side_ad = value; break;
      case VP_XY_SKEW_CALC:
        // now calcualte the skew factor
        xy_calib_data.xy_skew_factor = (float)_SKEW_FACTOR((float)xy_calib_data.xy_diag_ac, (float)xy_calib_data.xy_diag_bd, (float)xy_calib_data.xy_side_ad);
        ExtUI::setSkewFactor_xy(xy_calib_data.xy_skew_factor);
        (void)settings.save();
        GotoScreen(DGUSLCD_SCREEN_SKEW_CALIBRATION);
        break;
      default: return;
    }
  }
#endif

#if ENABLED(DGUS_FILAMENT_LOADUNLOAD)

  typedef struct  {
    ExtUI::extruder_t extruder; // which extruder to operate
    uint8_t action; // load or unload
    uint8_t target_temp; // accepted target temp  //Elsan
    uint8_t material_type;  //Elsan
    bool heated; // heating done ?
    float purge_length; // the length to extrude before unload, prevent filament jam
    float processed_length; //Elsan
    xyze_pos_t resume_position; //Elsan
    bool waiting_confirmation;  //Elsan
  } filament_data_t;

  static filament_data_t filament_data;
  static filament_data_t calibration_data;  //Elsan from old version

  //Elsan new version
  /*
  void DGUSScreenHandler::HandleFilamentOption(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleFilamentOption");

    uint8_t e_temp = 0;
    filament_data.heated = false;
    uint16_t preheat_option = swap16(*(uint16_t*)val_ptr);
    if (preheat_option <= 8)          // Load filament type
      filament_data.action = 1;
    else if (preheat_option >= 10) {  // Unload filament type
      preheat_option -= 10;
      filament_data.action = 2;
      filament_data.purge_length = DGUS_FILAMENT_PURGE_LENGTH;
    }
    else                              // Cancel filament operation
      filament_data.action = 0;

    switch (preheat_option) {
      case 0: // Load PLA
        #ifdef PREHEAT_1_TEMP_HOTEND
          e_temp = PREHEAT_1_TEMP_HOTEND;
        #endif
        break;
      case 1: // Load ABS
        TERN_(PREHEAT_2_TEMP_HOTEND, e_temp = PREHEAT_2_TEMP_HOTEND);
        break;
      case 2: // Load PET
        #ifdef PREHEAT_3_TEMP_HOTEND
          e_temp = PREHEAT_3_TEMP_HOTEND;
        #endif
        break;
      case 3: // Load FLEX
        #ifdef PREHEAT_4_TEMP_HOTEND
          e_temp = PREHEAT_4_TEMP_HOTEND;
        #endif
        break;
      case 9: // Cool down
      default:
        e_temp = 0;
        break;
    }

    if (filament_data.action == 0) { // Go back to utility screen
      #if HOTENDS >= 1
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
      #endif
      #if HOTENDS >= 2
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E1);
      #endif
      GotoScreen(DGUSLCD_SCREEN_UTILITY);
    }
    else { // Go to the preheat screen to show the heating progress
      switch (var.VP) {
        default: return;
        #if HOTENDS >= 1
          case VP_E0_FILAMENT_LOAD_UNLOAD:
            filament_data.extruder = ExtUI::extruder_t::E0;
            thermalManager.setTargetHotend(e_temp, filament_data.extruder);
            break;
        #endif
        #if HOTENDS >= 2
          case VP_E1_FILAMENT_LOAD_UNLOAD:
            filament_data.extruder = ExtUI::extruder_t::E1;
            thermalManager.setTargetHotend(e_temp, filament_data.extruder);
          break;
        #endif
      }
      GotoScreen(DGUSLCD_SCREEN_FILAMENT_HEATING);
    }
  }
  */

  //Elsan old version from Xlite.
  void DGUSScreenHandler::HandleFilamentOption(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleFilamentOption");

    uint8_t e_temp = 0;
    filament_data.heated = false;
    filament_data.processed_length = 0;
    filament_data.waiting_confirmation = false;
    uint16_t preheat_option = swap16(*(uint16_t*)val_ptr);
    if (preheat_option <= 8)          // Load filament type
      filament_data.action = 1;
    else if (preheat_option >= 20) {  // Purge filament type
      preheat_option = ExtUI::getMaterialType(); // get from eeprom
      filament_data.action = 3;
    } else if (preheat_option >= 10) {  // Unload filament type
      preheat_option = ExtUI::getMaterialType(); // get from eeprom
      filament_data.action = 2;
      filament_data.purge_length = DGUS_FILAMENT_PURGE_LENGTH;

      queue.inject_P(PSTR("G92 E0"));     //Elsan reset E motor position.
    } else                              // Cancel filament operation
      filament_data.action = 0;

    switch (preheat_option) {
      case 0: // Load PLA
        e_temp = PREHEAT_PLA_HOTEND;
        filament_data.material_type = 0;
        break;
      case 1: // Load ABS
        e_temp = PREHEAT_ABS_HOTEND;
        filament_data.material_type = 1;
        break;
      case 2: // Load PET
        e_temp = PREHEAT_PETG_HOTEND;
        filament_data.material_type = 2;
        break;
      case 3: // Load FLEX
        e_temp = PREHEAT_FLEX_HOTEND;
        filament_data.material_type = 3;
        break;
      case 7: // Load Custom
        filament_data.material_type = 7;
        e_temp = ExtUI::getMaterialCustomExtTemp();
        break;
      case 9: // Cool down
      default:
        e_temp = 0;
        break;
    }

    if (filament_data.action == 0) { // Go back to utility screen
      #if HOTENDS >= 1
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
      #endif
      #if HOTENDS >= 2
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E1);
      #endif
      GotoScreen(DGUSLCD_SCREEN_UTILITY);
    } else { // Go to the preheat screen to show the heating progress
      switch (var.VP) {
        default: return;
        #if HOTENDS >= 1
          case VP_E0_FILAMENT_LOAD_UNLOAD:
            filament_data.extruder = ExtUI::extruder_t::E0;
            thermalManager.setTargetHotend(e_temp, filament_data.extruder);
            break;
        #endif
        #if HOTENDS >= 2
          case VP_E1_FILAMENT_LOAD_UNLOAD:
            filament_data.extruder = ExtUI::extruder_t::E1;
            thermalManager.setTargetHotend(e_temp, filament_data.extruder);
          break;
        #endif
      }
      filament_data.target_temp = thermalManager.temp_hotend[filament_data.extruder].target;
      GotoScreen(DGUSLCD_SCREEN_FILAMENT_HEATING);
    }
  }

  //Elsan new version.  
  /*
  void DGUSScreenHandler::HandleFilamentLoadUnload(DGUS_VP_Variable &var) {
    DEBUG_ECHOLNPGM("HandleFilamentLoadUnload");
    if (filament_data.action <= 0) return;

    // If we close to the target temperature, we can start load or unload the filament
    if (thermalManager.hotEnoughToExtrude(filament_data.extruder) && \
       thermalManager.targetHotEnoughToExtrude(filament_data.extruder)) {
      float movevalue = DGUS_FILAMENT_LOAD_LENGTH_PER_TIME;

      if (filament_data.action == 1) { // load filament
        if (!filament_data.heated) {
          GotoScreen(DGUSLCD_SCREEN_FILAMENT_LOADING);
          filament_data.heated = true;
        }
        movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder)+movevalue;
      }
      else { // unload filament
        if (!filament_data.heated) {
          GotoScreen(DGUSLCD_SCREEN_FILAMENT_UNLOADING);
          filament_data.heated = true;
        }
        // Before unloading extrude to prevent jamming
        if (filament_data.purge_length >= 0) {
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
          filament_data.purge_length -= movevalue;
        }
        else
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) - movevalue;
      }
      ExtUI::setAxisPosition_mm(movevalue, filament_data.extruder);
    }
  }
  */

  //Elsan version from previous Xlite.
  void DGUSScreenHandler::HandleFilamentLoadUnload(DGUS_VP_Variable &var) {
    DEBUG_ECHOLNPGM("HandleFilamentLoadUnload");
    if (filament_data.action <= 0 || filament_data.waiting_confirmation) return;

    if (thermalManager.temp_hotend[filament_data.extruder].celsius >= filament_data.target_temp ||
        filament_data.heated) { // meaning if already got pass this check don't bother checking anymore!
      float movevalue = DGUS_FILAMENT_LOAD_LENGTH_PER_TIME * 3;

      if (((filament_data.action == 1 || filament_data.action == 2) && filament_data.processed_length > DGUS_FILAMENT_END_AFTER) ||
          (filament_data.action == 3 && filament_data.processed_length > DGUS_FILAMENT_PURGE_END_AFTER)) { // stop loading/unloading after...
        if (filament_data.action == 1 || filament_data.action == 3) { // if loading filament or purging
          filament_data.waiting_confirmation = true;
          // If the user won't confirm filament loading will continue on slow phase
          filament_data.processed_length = filament_data.action == 3
            ? 0 // Purge resets
            : DGUS_FILAMENT_LOAD_SLOW_AFTER + 1; // load

          GotoScreen(DGUSLCD_SCREEN_FILAMENT_CONFIRM);
          return; // will go to confirm no need to continue...
        }

        // user confirmed color... end...
        filament_data.action = 0;
        #if HOTENDS >= 1
          thermalManager.setTargetHotend(0, ExtUI::extruder_t::E0);
        #endif
        #if HOTENDS >= 2
          thermalManager.setTargetHotend(0, ExtUI::extruder_t::E1);
        #endif

        //filament_data.processed_length=0; //Elsan
        //filament_data.purge_length = 0;   //Elsan
        //queue.inject_P(PSTR("G92 E0"));     //Elsan reset E motor position.
        GotoScreen(DGUSLCD_SCREEN_UTILITY);
        return;
      }

      // we are ready set heated and go to loading screen...
      if (!filament_data.heated) {
        filament_data.heated = true;
        if (filament_data.action == 1) { // we are saving type if loading...
          ExtUI::setMaterialType(filament_data.material_type);
          (void)settings.save(); // save filament type to eeprom //Elsan do we need this?
        }
      }

      if (filament_data.action == 1) { // load filament
        GotoScreen(DGUSLCD_SCREEN_FILAMENT_LOADING);
        if (filament_data.processed_length > DGUS_FILAMENT_LOAD_SLOW_AFTER) movevalue /= 4; // slower after...
        filament_data.processed_length += movevalue; // keep track of length
        movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
      } else if (filament_data.action == 3) { // purge filament
        GotoScreen(DGUSLCD_SCREEN_FILAMENT_LOADING);
        movevalue /= 4; // slow always while purging and a bit more slower...
        filament_data.processed_length += movevalue; // keep track of length
        movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
      } else { // unload filament
        // Before unloading extrude to prevent jamming
        GotoScreen(DGUSLCD_SCREEN_FILAMENT_UNLOADING);
        if (filament_data.purge_length >= 0) {
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
          filament_data.purge_length -= movevalue;
        } else {
          filament_data.processed_length += movevalue; // keep track of length
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) - movevalue;
        }
      }
      ExtUI::setAxisPosition_mm(movevalue, filament_data.extruder);
    }
  }

  //Elsan
  void DGUSScreenHandler::HandleSDFilamentOption(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM(" HandleSDFilamentOption");
    uint16_t option = swap16(*(uint16_t*)val_ptr);
    filament_data.processed_length = 0;
    if (option <= 8)          // Load filament type
      filament_data.action = 1;
    else if (option >= 20)    // Purge filament type
      filament_data.action = 3;
    else if (option >= 10)    // Unload filament type
      filament_data.action = 2;
    else                      // Cancel filament operation
      filament_data.action = 0;

    if (filament_data.action == 0) { // Go back to utility screen
      GotoScreen(DGUSLCD_SCREEN_SDUTILITY);
      // reset e position where it was before
      ExtUI::setCurrentExtruderPosition_mm(filament_data.resume_position.e);
    } else {
      // filament load/unload clearance
      filament_data.resume_position = current_position;
    }
    switch (var.VP) {
      default: return;
      #if HOTENDS >= 1
        case VP_E0_FILAMENT_SD_LOAD_UNLOAD:
          filament_data.extruder = ExtUI::extruder_t::E0;
          break;
      #endif
      #if HOTENDS >= 2
        case VP_E1_FILAMENT_SD_LOAD_UNLOAD:
          filament_data.extruder = ExtUI::extruder_t::E1;
        break;
      #endif
    }
  }

  void DGUSScreenHandler::HandleSDFilamentLoadUnload(DGUS_VP_Variable &var) {
    if (filament_data.action <= 0) return;

    float movevalue = DGUS_FILAMENT_LOAD_LENGTH_PER_TIME * 3;

    if (((filament_data.action == 1 || filament_data.action == 2) && filament_data.processed_length > DGUS_FILAMENT_END_AFTER) ||
         (filament_data.action == 3 && filament_data.processed_length > DGUS_FILAMENT_PURGE_END_AFTER)) { // stop loading/unloading after...
      filament_data.action = 0;
      GotoScreen(DGUSLCD_SCREEN_SDUTILITY);
      return;
    }

    if (filament_data.action == 1) { // load filament
      if (filament_data.processed_length > DGUS_FILAMENT_LOAD_SLOW_AFTER) movevalue /= 4; // slower after 250mm
      filament_data.processed_length += movevalue; // keep track of length
      movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
    } else if (filament_data.action == 3) { // purge filament
      movevalue /= 4; // slow always while purging and a bit more slower...
      filament_data.processed_length += movevalue; // keep track of length
      movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
    } else { // unload filament
      // Before unloading extrude to prevent jamming
      filament_data.processed_length += movevalue; // keep track of length
      movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) - movevalue;
    }
    ExtUI::setAxisPosition_mm(movevalue, filament_data.extruder);
  }
#endif

//Elsan

#if ENABLED(FIRST_LAYER_CALIBRATION)
  void DGUSScreenHandler::HandleFirstLayerCalibrationOption(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleFirstLayerCalibrationOption");
    UNUSED(var);
    uint8_t e_temp = 0;
    uint8_t bed_temp = 0;
    calibration_data.action = 1;
    const uint16_t preheat_option = swap16(*(uint16_t*)val_ptr);

    switch (preheat_option) {
      case 0: // PLA
        e_temp = PREHEAT_PLA_HOTEND;
        bed_temp = PREHEAT_PLA_BED;
        break;
      case 1: // ABS
        e_temp = PREHEAT_ABS_HOTEND;
        bed_temp = PREHEAT_ABS_BED;
        break;
      case 2: // PETG
        e_temp = PREHEAT_PETG_HOTEND;
        bed_temp = PREHEAT_PETG_BED;
        break;
      case 3: // FLEX
        e_temp = PREHEAT_FLEX_HOTEND;
        bed_temp = PREHEAT_FLEX_BED;
        break;
      case 7: // Custom
        e_temp = ExtUI::getMaterialCustomExtTemp();
        bed_temp = ExtUI::getMaterialCustomBedTemp();
        break;
      case 9: // Cool down
      default:
        e_temp = 0;
        bed_temp = 0;
        calibration_data.action = 0;
        break;
    }

    thermalManager.setTargetHotend(e_temp, 0);
    thermalManager.setTargetBed(bed_temp);
    calibration_data.target_temp = e_temp;

    if (calibration_data.action == 0) { // Go back to calibration screen
      GotoScreen(DGUSLCD_SCREEN_CALIBRATION);
    } else {
      GotoScreen(DGUSLCD_SCREEN_CALIBRATION_HEATING);
      //SERIAL_ECHO_MSG("going to heating page");
    }
  }

  void DGUSScreenHandler::HandleFirstLayerCalibration(DGUS_VP_Variable &var) {
    DEBUG_ECHOLNPGM("HandleFirstLayerCalibration");
    if (calibration_data.action <= 0) return;
    if (thermalManager.temp_hotend[0].celsius > 160 && calibration_data.action == 1) {
        // Do a G29 here to speed up the process
        queue.inject_P(PSTR("M107\nG90\nM83\nG21\nG90\nM83\nG28\nG29"));
        calibration_data.action = 2; // run the first layer cal code when ready
    } else if (thermalManager.temp_hotend[0].celsius >= calibration_data.target_temp && calibration_data.action == 2) {
      GotoScreen(DGUSLCD_SCREEN_FIRST_LAYER_CAL);
      queue.inject_P(PSTR("G40"));
      calibration_data.action = 0; // Do this once
    }
  }
#endif

//Elsan
void DGUSScreenHandler::HandleFilamentConfirmation(DGUS_VP_Variable &var, void *val_ptr) {
    UNUSED(var); UNUSED(val_ptr);
    filament_data.waiting_confirmation = false;
}

void DGUSScreenHandler::HandleFilamentInit(DGUS_VP_Variable &var, void *val_ptr) {
    UNUSED(var); UNUSED(val_ptr);
    queue.inject_P(PSTR("G28 O\nG90\nG1 X10 Y0 Z100 F4200"));
    // No need to move home. Just go up 30
    //queue.inject_P(PSTR("G91\nG1 Z30 F4200\nG90"));
  }

void DGUSScreenHandler::HandleGoToCalibrationPoints(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleGoToCalibrationPoints");
  queue.inject_P(PSTR("G28\nG29"));
}

void DGUSScreenHandler::HandleGoToMaximumPoints(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleGoToMaximumPoints");

  char buf[32] = {0};
  /*
  queue.inject_P(PSTR("G28"));
  sprintf(buf, "G1 Z%d F4200", Z_MAX_POS);
  queue.enqueue_one_now(buf);
  sprintf(buf, "G1 X%d Y%d F4200", 0, 0);
  queue.enqueue_one_now(buf);
  sprintf(buf, "G1 X%d Y%d F4200", 0, Y_BED_SIZE);
  queue.enqueue_one_now(buf);
  sprintf(buf, "G1 X%d Y%d F4200", X_BED_SIZE, Y_BED_SIZE);
  queue.enqueue_one_now(buf);
  sprintf(buf, "G1 X%d Y%d Z5 F4200", X_BED_SIZE, 0);
  queue.enqueue_one_now(buf);
  */

  queue.inject_P(PSTR("G28"));
  //queue.enqueue_one_now(PSTR("G28")); //Elsan
  
  sprintf(buf, "G0 Z%d F4200", Z_MAX_POS);
  queue.enqueue_one_now(buf);
  
  sprintf(buf, "G0 X%d Y%d F4200", 0, 0);
  queue.enqueue_one_now(buf);
  
  sprintf(buf, "G0 X%d Y%d F4200", 0, Y_BED_SIZE);  //Works
  //snprintf_P(buf, 32, PSTR("G0 X%d Y%d F4200"), 0, Y_BED_SIZE);
  queue.enqueue_one_now(buf);
  
  buf[0]=0;
  sprintf(buf, "G0 X%d Y%d F4200", X_BED_SIZE, Y_BED_SIZE);
  queue.enqueue_one_now(buf); //Crash
  //queue.enqueue_one_P(buf);
  //gcode.process_subcommands_now_P(buf); //problematic
  //queue.enqueue_one_now(PSTR("G0 X190 Y190 F4200"));  //Works  
  //HAL_Delay(100); //Test
  
  buf[0]=0;
  //sprintf(buf, "G0 X%d Y%d Z5 F4200", X_BED_SIZE, 0);
  //queue.enqueue_one_now(buf); //Crash
  //queue.enqueue_one_P(buf);
  //queue.enqueue_now_P(buf); //Crash
  //queue.inject_P(buf);
  //queue.inject_P(PSTR("G0 X190 Y190 Z5 F4200"));  //Bad
  //queue.enqueue_one_now(PSTR("G0 X190 Y190 Z5 F4200")); //Disable for the time being. It seems that queue is not enough.
}

bool fanStatus = false;
void DGUSScreenHandler::HandleFanOnOff(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleFanOnOff");

  char buf[5];
  strcpy_P(buf, fanStatus ? PSTR("M107") : PSTR("M106"));
  fanStatus = !fanStatus;

  queue.enqueue_one_now(buf);
}
/////////////////////////////////////////////////////////////////////////////////

void DGUSScreenHandler::UpdateNewScreen(DGUSLCD_Screens newscreen, bool popup) {
  DEBUG_ECHOLNPAIR("SetNewScreen: ", newscreen);

  if (!popup) {
    memmove(&past_screens[1], &past_screens[0], sizeof(past_screens) - 1);
    past_screens[0] = current_screen;
  }

  current_screen = newscreen;
  skipVP = 0;
  ForceCompleteUpdate();
}

void DGUSScreenHandler::PopToOldScreen() {
  DEBUG_ECHOLNPAIR("PopToOldScreen s=", past_screens[0]);
  GotoScreen(past_screens[0], true);
  memmove(&past_screens[0], &past_screens[1], sizeof(past_screens) - 1);
  past_screens[sizeof(past_screens) - 1] = DGUSLCD_SCREEN_MAIN;
}

void DGUSScreenHandler::UpdateScreenVPData() {
  DEBUG_ECHOPAIR(" UpdateScreenVPData Screen: ", current_screen);

  const uint16_t *VPList = DGUSLCD_FindScreenVPMapList(current_screen);
  if (!VPList) {
    DEBUG_ECHOLNPAIR(" NO SCREEN FOR: ", current_screen);
    ScreenComplete = true;
    return;  // nothing to do, likely a bug or boring screen.
  }

  // Round-robin updating of all VPs.
  VPList += update_ptr;

  bool sent_one = false;
  do {
    uint16_t VP = pgm_read_word(VPList);
    DEBUG_ECHOPAIR(" VP: ", VP);
    if (!VP) {
      update_ptr = 0;
      DEBUG_ECHOLNPGM(" UpdateScreenVPData done");
      ScreenComplete = true;
      return;  // Screen completed.
    }

    if (VP == skipVP) { skipVP = 0; continue; }

    DGUS_VP_Variable rcpy;
    if (populate_VPVar(VP, &rcpy)) {
      uint8_t expected_tx = 6 + rcpy.size;  // expected overhead is 6 bytes + payload.
      // Send the VP to the display, but try to avoid overrunning the Tx Buffer.
      // But send at least one VP, to avoid getting stalled.
      if (rcpy.send_to_display_handler && (!sent_one || expected_tx <= dgusdisplay.GetFreeTxBuffer())) {
        //DEBUG_ECHOPAIR(" calling handler for ", rcpy.VP);
        sent_one = true;
        rcpy.send_to_display_handler(rcpy);
      }
      else {
        //auto x=dgusdisplay.GetFreeTxBuffer();
        //DEBUG_ECHOLNPAIR(" tx almost full: ", x);
        //DEBUG_ECHOPAIR(" update_ptr ", update_ptr);
        ScreenComplete = false;
        return;  // please call again!
      }
    }

  } while (++update_ptr, ++VPList, true);
}

void DGUSScreenHandler::GotoScreen(DGUSLCD_Screens screen, bool ispopup) {
  dgusdisplay.RequestScreen(screen);
  UpdateNewScreen(screen, ispopup);
}

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
      GotoScreen(DGUSLCD_SCREEN_MAIN);
    }
  #endif
  return IsScreenComplete();
}

void DGUSDisplay::RequestScreen(DGUSLCD_Screens screen) {
  DEBUG_ECHOLNPAIR("GotoScreen ", screen);
  const unsigned char gotoscreen[] = { 0x5A, 0x01, (unsigned char) (screen >> 8U), (unsigned char) (screen & 0xFFU) };
  WriteVariable(0x84, gotoscreen, sizeof(gotoscreen));
}

#endif // HAS_DGUS_LCD
