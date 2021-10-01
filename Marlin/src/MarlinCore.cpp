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

/**
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/grbl/grbl
 */

#include "MarlinCore.h"

#if ENABLED(MARLIN_DEV_MODE)
  #warning "WARNING! Disable MARLIN_DEV_MODE for the final build!"
#endif

#include "HAL/shared/Delay.h"
#include "HAL/shared/esp_wifi.h"

#ifdef ARDUINO
  #include <pins_arduino.h>
#endif
#include <math.h>

#include "core/utility.h"
#include "module/motion.h"
#include "module/planner.h"
#include "module/endstops.h"
#include "module/temperature.h"
#include "module/settings.h"
#include "module/printcounter.h" // PrintCounter or Stopwatch

#include "module/stepper.h"
#include "module/stepper/indirection.h"

#include "gcode/gcode.h"
#include "gcode/parser.h"
#include "gcode/queue.h"

#include "sd/cardreader.h"

#include "lcd/ultralcd.h"

//Elsan
#include "main.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "app_ethernet.h"
#include "httpserver.h"
struct netif gnetif;
static void Netif_Config(void);
extern "C" void IAP_httpd_init(void);
extern "C" void ethernetif_input(struct netif *netif);
extern "C" err_t ethernetif_init(struct netif *netif);
extern "C" void ethernetif_update_config(struct netif *netif);

//#define HAS_X_ENABLE 1
//#define HAS_X_DIR 1
//#define HAS_X_STEP 1
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
void HAL_SPI_MspInitTEST(void);
void usb_check(void);
//extern "C" void SPI3_IRQHandler(void);
#include "SPI.h"
//#include <Arduino.h>
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
char buff [50];
//volatile byte indx;
//volatile boolean process;
SPI_HandleTypeDef SpiHandle;
static SPI_HandleTypeDef spi = { .Instance = SPI3 };
uint8_t aTxBuffer[] = "*SPI*";
/* Buffer used for reception */
uint8_t aRxBuffer[100];
char output[100];
unsigned char bufSPI[200];
unsigned char printBuf[20];
unsigned char start=0;
unsigned long int say=0;
unsigned char bufASCI,bufInvASCI;
unsigned char bufASCIp;
long int ASCIcnt=0;
int ASCIcame=0;
int InvASCIcame=0;
char write_perm=1;
extern char fname2[/*20*/100];
void usb_file_open_wr(void);
void usb_ls2(void);
#include "FATFS/App/fatfs.h"
extern FIL MyFile;
inline void SPI3_Transfer(uint8_t *outp, uint8_t *inp, int count);
int ftp_name_wr=1;
char fname_ftp[100];
int fname_cnt=0;
extern char isUSB_fileopen;
extern char buf_main[50][200];
extern int print_stat;
extern int x_pos, y_pos, z_pos;
//Fan
int tach_first=1;
int tach2=0;
int cntr=0;
int activity=0;
int loop_speed=100;
int first_trig=0;
int sec_trig=0;
uint32_t m, m2;
uint32_t us,us2;
int tach_first0=1;
int tach20=0;
int cntr0=0;
int activity0=0;
int loop_speed0=100;
int first_trig0=0;
int sec_trig0=0;
//uint32_t m, m2;
uint32_t us0,us20;
extern char noUSB;
int USB_check_cntr; //Counter variables should be put here not in main loop.
#include "lcd/extui/lib/dgus/fysetc/DGUSDisplayDef.h"
#include "lcd/extui/lib/dgus/DGUSScreenHandler.h"
extern DGUSScreenHandler ScreenHandler; //Elsan
extern char USB_check_sec;

#define SPIx                             SPI3
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI3_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI3_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI3_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_10
#define SPIx_SCK_GPIO_PORT               GPIOC
#define SPIx_SCK_AF                      GPIO_AF6_SPI3 //GPIO_AF5_SPI2

#define SPIx_MISO_PIN                    GPIO_PIN_11
#define SPIx_MISO_GPIO_PORT              GPIOC
#define SPIx_MISO_AF                     GPIO_AF6_SPI3 //GPIO_AF5_SPI2

#define SPIx_MOSI_PIN                    GPIO_PIN_12
#define SPIx_MOSI_GPIO_PORT              GPIOC
#define SPIx_MOSI_AF                     GPIO_AF6_SPI3 //GPIO_AF5_SPI2

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI3_IRQn
#define SPIx_IRQHandler                  SPI3_IRQHandler
/////////////////////////////////////////////////////////////////////////

#if HAS_TOUCH_XPT2046
  #include "lcd/touch/touch_buttons.h"
#endif

#if HAS_TFT_LVGL_UI
  #include "lcd/extui/lib/mks_ui/tft_lvgl_configuration.h"
  #include "lcd/extui/lib/mks_ui/draw_ui.h"
  #include "lcd/extui/lib/mks_ui/mks_hardware_test.h"
  #include <lvgl.h>
#endif

#if ENABLED(DWIN_CREALITY_LCD)
  #include "lcd/dwin/e3v2/dwin.h"
  #include "lcd/dwin/dwin_lcd.h"
  #include "lcd/dwin/e3v2/rotary_encoder.h"
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #include "libs/BL24CXX.h"
#endif

#if ENABLED(DIRECT_STEPPING)
  #include "feature/direct_stepping.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "feature/host_actions.h"
#endif

#if USE_BEEPER
  #include "libs/buzzer.h"
#endif

#if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
  #include "feature/closedloop.h"
#endif

#if HAS_MOTOR_CURRENT_I2C
  #include "feature/digipot/digipot.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "feature/mixing.h"
#endif

#if ENABLED(MAX7219_DEBUG)
  #include "feature/max7219.h"
#endif

#if HAS_COLOR_LEDS
  #include "feature/leds/leds.h"
#endif

#if ENABLED(BLTOUCH)
  #include "feature/bltouch.h"
#endif

#if ENABLED(POLL_JOG)
  #include "feature/joystick.h"
#endif

#if HAS_SERVOS
  #include "module/servo.h"
#endif

#if ENABLED(HAS_MOTOR_CURRENT_DAC)
  #include "feature/dac/stepper_dac.h"
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "feature/twibus.h"
  TWIBus i2c;
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "feature/encoder_i2c.h"
#endif

#if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
  #include "feature/tmc_util.h"
#endif

#if HAS_CUTTER
  #include "feature/spindle_laser.h"
#endif

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif

#if ENABLED(G38_PROBE_TARGET)
  uint8_t G38_move; // = 0
  bool G38_did_trigger; // = false
#endif

#if ENABLED(DELTA)
  #include "module/delta.h"
#elif IS_SCARA
  #include "module/scara.h"
#endif

#if HAS_LEVELING
  #include "feature/bedlevel/bedlevel.h"
#endif

#if BOTH(ADVANCED_PAUSE_FEATURE, PAUSE_PARK_NO_STEPPER_TIMEOUT)
  #include "feature/pause.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "feature/cancel_object.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "feature/runout.h"
#endif

#if HAS_Z_SERVO_PROBE
  #include "module/probe.h"
#endif

#if ENABLED(HOTEND_IDLE_TIMEOUT)
  #include "feature/hotend_idle.h"
#endif

#if ENABLED(TEMP_STAT_LEDS)
  #include "feature/leds/tempstat.h"
#endif

#if ENABLED(CASE_LIGHT_ENABLE)
  #include "feature/caselight.h"
#endif

#if HAS_FANMUX
  #include "feature/fanmux.h"
#endif

#if DO_SWITCH_EXTRUDER || ANY(SWITCHING_NOZZLE, PARKING_EXTRUDER, MAGNETIC_PARKING_EXTRUDER, ELECTROMAGNETIC_SWITCHING_TOOLHEAD, SWITCHING_TOOLHEAD)
  #include "module/tool_change.h"
#endif

#if ENABLED(USE_CONTROLLER_FAN)
  #include "feature/controllerfan.h"
#endif

#if ENABLED(PRUSA_MMU2)
  #include "feature/mmu2/mmu2.h"
#endif

#if HAS_L64XX
  #include "libs/L64XX/L64XX_Marlin.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "feature/password/password.h"
#endif

PGMSTR(NUL_STR, "");
PGMSTR(M112_KILL_STR, "M112 Shutdown");
PGMSTR(G28_STR, "G28");
PGMSTR(M21_STR, "M21");
PGMSTR(M23_STR, "M23 %s");
PGMSTR(M24_STR, "M24");
PGMSTR(SP_P_STR, " P");  PGMSTR(SP_T_STR, " T");
PGMSTR(X_STR,     "X");  PGMSTR(Y_STR,     "Y");  PGMSTR(Z_STR,     "Z");  PGMSTR(E_STR,     "E");
PGMSTR(X_LBL,     "X:"); PGMSTR(Y_LBL,     "Y:"); PGMSTR(Z_LBL,     "Z:"); PGMSTR(E_LBL,     "E:");
PGMSTR(SP_A_STR, " A");  PGMSTR(SP_B_STR, " B");  PGMSTR(SP_C_STR, " C");
PGMSTR(SP_X_STR, " X");  PGMSTR(SP_Y_STR, " Y");  PGMSTR(SP_Z_STR, " Z");  PGMSTR(SP_E_STR, " E");
PGMSTR(SP_X_LBL, " X:"); PGMSTR(SP_Y_LBL, " Y:"); PGMSTR(SP_Z_LBL, " Z:"); PGMSTR(SP_E_LBL, " E:");

MarlinState marlin_state = MF_INITIALIZING;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
bool wait_for_heatup = true;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if HAS_RESUME_CONTINUE
  bool wait_for_user; // = false;

  void wait_for_user_response(millis_t ms/*=0*/, const bool no_sleep/*=false*/) {
    TERN(ADVANCED_PAUSE_FEATURE,,UNUSED(no_sleep));
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    wait_for_user = true;
    if (ms) ms += millis(); // expire time
    while (wait_for_user && !(ms && ELAPSED(millis(), ms)))
      idle(TERN_(ADVANCED_PAUSE_FEATURE, no_sleep));
      //idle(); //Elsan test
    wait_for_user = false;
  }

#endif

#if PIN_EXISTS(CHDK)
  extern millis_t chdk_timeout;
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  I2CPositionEncodersMgr I2CPEM;
#endif

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void setup_killpin() {
  #if HAS_KILL
    #if KILL_PIN_STATE
      SET_INPUT_PULLDOWN(KILL_PIN);
    #else
      SET_INPUT_PULLUP(KILL_PIN);
    #endif
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, !SUICIDE_PIN_INVERTING);
  #endif
  #if ENABLED(PSU_CONTROL)
    powersupply_on = ENABLED(PSU_DEFAULT_OFF);
    if (ENABLED(PSU_DEFAULT_OFF)) PSU_OFF(); else PSU_ON();
  #endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() { OUT_WRITE(STEPPER_RESET_PIN, LOW); } // Drive down to keep motor driver chips in reset
  void enableStepperDrivers()  { SET_INPUT(STEPPER_RESET_PIN); }      // Set to input, allowing pullups to pull the pin high
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0

  void i2c_on_receive(int bytes) { // just echo all bytes received to serial
    i2c.receive(bytes);
  }

  void i2c_on_request() {          // just send dummy data for now
    i2c.reply("Hello World!\n");
  }

#endif

/**
 * Sensitive pin test for M42, M226
 */

#include "pins/sensitive_pins.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"

bool pin_is_protected(const pin_t pin) {
  static const pin_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  LOOP_L_N(i, COUNT(sensitive_pins)) {
    pin_t sensitive_pin;
    memcpy_P(&sensitive_pin, &sensitive_pins[i], sizeof(pin_t));
    if (pin == sensitive_pin) return true;
  }
  return false;
}

#pragma GCC diagnostic pop

void protected_pin_err() {
  SERIAL_ERROR_MSG(STR_ERR_PROTECTED_PIN);
}

void quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES);
  sync_plan_position();
}

void enable_e_steppers() {
  #define _ENA_E(N) ENABLE_AXIS_E##N();
  REPEAT(E_STEPPERS, _ENA_E)
}

void enable_all_steppers() {
  TERN_(AUTO_POWER_CONTROL, powerManager.power_on());
  ENABLE_AXIS_X();
  ENABLE_AXIS_Y();
  ENABLE_AXIS_Z();
  enable_e_steppers();
}

void disable_e_steppers() {
  #define _DIS_E(N) DISABLE_AXIS_E##N();
  REPEAT(E_STEPPERS, _DIS_E)
}

void disable_e_stepper(const uint8_t e) {
  #define _CASE_DIS_E(N) case N: DISABLE_AXIS_E##N(); break;
  switch (e) {
    REPEAT(EXTRUDERS, _CASE_DIS_E)
  }
}

void disable_all_steppers() {
  DISABLE_AXIS_X();
  DISABLE_AXIS_Y();
  DISABLE_AXIS_Z();
  disable_e_steppers();
}

#if ENABLED(G29_RETRY_AND_RECOVER)

  void event_probe_failure() {
    #ifdef ACTION_ON_G29_FAILURE
      host_action(PSTR(ACTION_ON_G29_FAILURE));
    #endif
    #ifdef G29_FAILURE_COMMANDS
      gcode.process_subcommands_now_P(PSTR(G29_FAILURE_COMMANDS));
    #endif
    #if ENABLED(G29_HALT_ON_FAILURE)
      #ifdef ACTION_ON_CANCEL
        host_action_cancel();
      #endif
      kill(GET_TEXT(MSG_LCD_PROBING_FAILED));
    #endif
  }

  void event_probe_recover() {
    TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_INFO, PSTR("G29 Retrying"), DISMISS_STR));
    #ifdef ACTION_ON_G29_RECOVER
      host_action(PSTR(ACTION_ON_G29_RECOVER));
    #endif
    #ifdef G29_RECOVER_COMMANDS
      gcode.process_subcommands_now_P(PSTR(G29_RECOVER_COMMANDS));
    #endif
  }

#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "feature/pause.h"
#else
  constexpr bool did_pause_print = false;
#endif

/**
 * A Print Job exists when the timer is running or SD printing
 */
bool printJobOngoing() {
  return print_job_timer.isRunning() || IS_SD_PRINTING();
}

/**
 * Printing is active when the print job timer is running
 */
bool printingIsActive() {
  return !did_pause_print && (print_job_timer.isRunning() || IS_SD_PRINTING());
}

/**
 * Printing is paused according to SD or host indicators
 */
bool printingIsPaused() {
  return did_pause_print || print_job_timer.isPaused() || IS_SD_PAUSED();
}

void startOrResumeJob() {
  if (!printingIsPaused()) {
    TERN_(CANCEL_OBJECTS, cancelable.reset());
    TERN_(LCD_SHOW_E_TOTAL, e_move_accumulator = 0);
    #if BOTH(LCD_SET_PROGRESS_MANUALLY, USE_M73_REMAINING_TIME)
      ui.reset_remaining_time();
    #endif
  }
  print_job_timer.start();
}

#if ENABLED(SDSUPPORT)

  inline void abortSDPrinting() {
    card.endFilePrint(TERN_(SD_RESORT, true));
    
    queue.clear();
    quickstop_stepper();
    print_job_timer.stop();
    #if DISABLED(SD_ABORT_NO_COOLDOWN)
      thermalManager.disable_all_heaters();
    #endif
    #if !HAS_CUTTER
      thermalManager.zero_fan_speeds();
    #else
      cutter.kill();              // Full cutter shutdown including ISR control
    #endif
    wait_for_heatup = false;
    //TERN_(POWER_LOSS_RECOVERY, recovery.purge()); //Elsan dis
    #ifdef EVENT_GCODE_SD_ABORT
      queue.inject_P(PSTR(EVENT_GCODE_SD_ABORT)); //Elsan dis G28XY will crash with no HW.
    #endif
    
    TERN_(PASSWORD_AFTER_SD_PRINT_ABORT, password.lock_machine());
  }

  inline void finishSDPrinting() {
    if (queue.enqueue_one_P(PSTR("M1001"))) {
      marlin_state = MF_RUNNING;
      TERN_(PASSWORD_AFTER_SD_PRINT_END, password.lock_machine());
    }
  }

#endif // SDSUPPORT

/**
 * Minimal management of Marlin's core activities:
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if CHDK_PIN needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - Pulse FET_SAFETY_PIN if it exists
 */
inline void manage_inactivity(const bool ignore_stepper_queue=false) {

  if (queue.length < BUFSIZE) queue.get_available_commands();

  const millis_t ms = millis();

  // Prevent steppers timing-out in the middle of M600
  // unless PAUSE_PARK_NO_STEPPER_TIMEOUT is disabled
  const bool parked_or_ignoring = ignore_stepper_queue ||
     (BOTH(ADVANCED_PAUSE_FEATURE, PAUSE_PARK_NO_STEPPER_TIMEOUT) && did_pause_print);

  // Reset both the M18/M84 activity timeout and the M85 max 'kill' timeout
  if (parked_or_ignoring) gcode.reset_stepper_timeout(ms);

  if (gcode.stepper_max_timed_out(ms)) {
    SERIAL_ERROR_START();
    SERIAL_ECHOLNPAIR(STR_KILL_INACTIVE_TIME, parser.command_ptr);
    kill();
  }

  // M18 / M94 : Handle steppers inactive time timeout
  if (gcode.stepper_inactive_time) {

    static bool already_shutdown_steppers; // = false

    // Any moves in the planner? Resets both the M18/M84
    // activity timeout and the M85 max 'kill' timeout
    if (planner.has_blocks_queued())
      gcode.reset_stepper_timeout(ms);
    else if (!parked_or_ignoring && gcode.stepper_inactive_timeout()) {
      if (!already_shutdown_steppers) {
        already_shutdown_steppers = true;  // L6470 SPI will consume 99% of free time without this

        // Individual axes will be disabled if configured
        if (ENABLED(DISABLE_INACTIVE_X)) DISABLE_AXIS_X();
        if (ENABLED(DISABLE_INACTIVE_Y)) DISABLE_AXIS_Y();
        if (ENABLED(DISABLE_INACTIVE_Z)) DISABLE_AXIS_Z();
        if (ENABLED(DISABLE_INACTIVE_E)) disable_e_steppers();

        TERN_(AUTO_BED_LEVELING_UBL, ubl.steppers_were_disabled());
      }
    }
    else
      already_shutdown_steppers = false;
  }

  #if PIN_EXISTS(CHDK) // Check if pin should be set to LOW (after M240 set it HIGH)
    if (chdk_timeout && ELAPSED(ms, chdk_timeout)) {
      chdk_timeout = 0;
      WRITE(CHDK_PIN, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (kill_state())
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) {
      SERIAL_ERROR_MSG(STR_KILL_BUTTON);
      kill();
    }
  #endif

  #if HAS_HOME
    // Handle a standalone HOME button
    constexpr millis_t HOME_DEBOUNCE_DELAY = 1000UL;
    static millis_t next_home_key_ms; // = 0
    if (!IS_SD_PRINTING() && !READ(HOME_PIN)) { // HOME_PIN goes LOW when pressed
      const millis_t ms = millis();
      if (ELAPSED(ms, next_home_key_ms)) {
        next_home_key_ms = ms + HOME_DEBOUNCE_DELAY;
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
        queue.enqueue_now_P(G28_STR);
      }
    }
  #endif

  TERN_(USE_CONTROLLER_FAN, controllerFan.update()); // Check if fan should be turned on to cool stepper drivers down

  TERN_(AUTO_POWER_CONTROL, powerManager.check());

  TERN_(HOTEND_IDLE_TIMEOUT, hotend_idle.check());

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP
      && ELAPSED(ms, gcode.previous_move_ms + SEC_TO_MS(EXTRUDER_RUNOUT_SECONDS))
      && !planner.has_blocks_queued()
    ) {
      #if ENABLED(SWITCHING_EXTRUDER)
        bool oldstatus;
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_READ(); ENABLE_AXIS_E0(); break;
          #if E_STEPPERS > 1
            case 2: case 3: oldstatus = E1_ENABLE_READ(); ENABLE_AXIS_E1(); break;
            #if E_STEPPERS > 2
              case 4: case 5: oldstatus = E2_ENABLE_READ(); ENABLE_AXIS_E2(); break;
              #if E_STEPPERS > 3
                case 6: case 7: oldstatus = E3_ENABLE_READ(); ENABLE_AXIS_E3(); break;
              #endif // E_STEPPERS > 3
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #else // !SWITCHING_EXTRUDER
        bool oldstatus;
        switch (active_extruder) {
          default:
          #define _CASE_EN(N) case N: oldstatus = E##N##_ENABLE_READ(); ENABLE_AXIS_E##N(); break;
          REPEAT(E_STEPPERS, _CASE_EN);
        }
      #endif

      const float olde = current_position.e;
      current_position.e += EXTRUDER_RUNOUT_EXTRUDE;
      line_to_current_position(MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED));
      current_position.e = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();

      #if ENABLED(SWITCHING_EXTRUDER)
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_WRITE(oldstatus); break;
          #if E_STEPPERS > 1
            case 2: case 3: oldstatus = E1_ENABLE_WRITE(oldstatus); break;
            #if E_STEPPERS > 2
              case 4: case 5: oldstatus = E2_ENABLE_WRITE(oldstatus); break;
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #else // !SWITCHING_EXTRUDER
        switch (active_extruder) {
          #define _CASE_RESTORE(N) case N: E##N##_ENABLE_WRITE(oldstatus); break;
          REPEAT(E_STEPPERS, _CASE_RESTORE);
        }
      #endif // !SWITCHING_EXTRUDER

      gcode.reset_stepper_timeout(ms);
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ELAPSED(ms, delayed_move_time + 1000UL) && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      destination = current_position;
      prepare_line_to_destination();
    }
  #endif

  TERN_(TEMP_STAT_LEDS, handle_status_leds());

  TERN_(MONITOR_DRIVER_STATUS, monitor_tmc_drivers());

  TERN_(MONITOR_L6470_DRIVER_STATUS, L64xxManager.monitor_driver());

  // Limit check_axes_activity frequency to 10Hz
  static millis_t next_check_axes_ms = 0;
  if (ELAPSED(ms, next_check_axes_ms)) {
    planner.check_axes_activity();
    next_check_axes_ms = ms + 100UL;
  }

  #if PIN_EXISTS(FET_SAFETY)
    static millis_t FET_next;
    if (ELAPSED(ms, FET_next)) {
      FET_next = ms + FET_SAFETY_DELAY;  // 2µs pulse every FET_SAFETY_DELAY mS
      OUT_WRITE(FET_SAFETY_PIN, !FET_SAFETY_INVERTED);
      DELAY_US(2);
      WRITE(FET_SAFETY_PIN, FET_SAFETY_INVERTED);
    }
  #endif
}

/**
 * Standard idle routine keeps the machine alive:
 *  - Core Marlin activities
 *  - Manage heaters (and Watchdog)
 *  - Max7219 heartbeat, animation, etc.
 *
 *  Only after setup() is complete:
 *  - Handle filament runout sensors
 *  - Run HAL idle tasks
 *  - Handle Power-Loss Recovery
 *  - Run StallGuard endstop checks
 *  - Handle SD Card insert / remove
 *  - Handle USB Flash Drive insert / remove
 *  - Announce Host Keepalive state (if any)
 *  - Update the Print Job Timer state
 *  - Update the Beeper queue
 *  - Read Buttons and Update the LCD
 *  - Run i2c Position Encoders
 *  - Auto-report Temperatures / SD Status
 *  - Update the Průša MMU2
 *  - Handle Joystick jogging
 */
void idle(TERN_(ADVANCED_PAUSE_FEATURE, bool no_stepper_sleep/*=false*/)) {

  // Core Marlin activities
  manage_inactivity(TERN_(ADVANCED_PAUSE_FEATURE, no_stepper_sleep));

  // Manage Heaters (and Watchdog)
  thermalManager.manage_heater();

  // Max7219 heartbeat, animation, etc
  //elsan dis
  //TERN_(MAX7219_DEBUG, max7219.idle_tasks());

  // Return if setup() isn't completed
  if (marlin_state == MF_INITIALIZING) return;

  // Handle filament runout sensors
  TERN_(HAS_FILAMENT_SENSOR, runout.run());

  // Run HAL idle tasks
  #ifdef HAL_IDLETASK
    HAL_idletask();
  #endif

  // Handle Power-Loss Recovery
  #if ENABLED(POWER_LOSS_RECOVERY) && PIN_EXISTS(POWER_LOSS)
    if (printJobOngoing()) recovery.outage();
  #endif

  // Run StallGuard endstop checks
  #if ENABLED(SPI_ENDSTOPS)
    if (endstops.tmc_spi_homing.any
      && TERN1(IMPROVE_HOMING_RELIABILITY, ELAPSED(millis(), sg_guard_period))
    ) LOOP_L_N(i, 4) // Read SGT 4 times per idle loop
        if (endstops.tmc_spi_homing_check()) break;
  #endif

  // Handle SD Card insert / remove
  //elsan dis
  //TERN_(SDSUPPORT, card.manage_media());

  // Handle USB Flash Drive insert / remove
  
  TERN_(USB_FLASH_DRIVE_SUPPORT, Sd2Card::idle());

  // Announce Host Keepalive state (if any)
  TERN_(HOST_KEEPALIVE_FEATURE, gcode.host_keepalive());

  // Update the Print Job Timer state
  TERN_(PRINTCOUNTER, print_job_timer.tick());

  // Update the Beeper queue
  TERN_(USE_BEEPER, buzzer.tick());

  // Handle UI input / draw events
  
  TERN(DWIN_CREALITY_LCD, DWIN_Update(), ui.update());

  // Run i2c Position Encoders
  #if ENABLED(I2C_POSITION_ENCODERS)
    static millis_t i2cpem_next_update_ms;
    if (planner.has_blocks_queued()) {
      const millis_t ms = millis();
      if (ELAPSED(ms, i2cpem_next_update_ms)) {
        I2CPEM.update();
        i2cpem_next_update_ms = ms + I2CPE_MIN_UPD_TIME_MS;
      }
    }
  #endif

  // Auto-report Temperatures / SD Status
  #if HAS_AUTO_REPORTING
    if (!gcode.autoreport_paused) {
      TERN_(AUTO_REPORT_TEMPERATURES, thermalManager.auto_report_temperatures());
      TERN_(AUTO_REPORT_SD_STATUS, card.auto_report_sd_status());
    }
  #endif

  // Update the Průša MMU2
  //elsan dis
  //TERN_(PRUSA_MMU2, mmu2.mmu_loop());

  // Handle Joystick jogging
  //elsan dis
  //TERN_(POLL_JOG, joystick.inject_jog_moves());

  // Direct Stepping
  TERN_(DIRECT_STEPPING, page_manager.write_responses());

  #if HAS_TFT_LVGL_UI
    LV_TASK_HANDLER();
  #endif
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(PGM_P const lcd_error/*=nullptr*/, PGM_P const lcd_component/*=nullptr*/, const bool steppers_off/*=false*/) {
  thermalManager.disable_all_heaters();

  TERN_(HAS_CUTTER, cutter.kill()); // Full cutter shutdown including ISR control

  SERIAL_ERROR_MSG(STR_ERR_KILLED);

  #if HAS_DISPLAY
    ui.kill_screen(lcd_error ?: GET_TEXT(MSG_KILLED), lcd_component ?: NUL_STR);
  #else
    UNUSED(lcd_error);
    UNUSED(lcd_component);
  #endif

  #if HAS_TFT_LVGL_UI
    lv_draw_error_message(lcd_error);
  #endif

  #ifdef ACTION_ON_KILL
    host_action_kill();
  #endif

  minkill(steppers_off);
}

void minkill(const bool steppers_off/*=false*/) {

  // Wait a short time (allows messages to get out before shutting down.
  for (int i = 1000; i--;) DELAY_US(600);

  cli(); // Stop interrupts

  // Wait to ensure all interrupts stopped
  for (int i = 1000; i--;) DELAY_US(250);

  // Reiterate heaters off
  thermalManager.disable_all_heaters();

  TERN_(HAS_CUTTER, cutter.kill());  // Reiterate cutter shutdown

  // Power off all steppers (for M112) or just the E steppers
  steppers_off ? disable_all_steppers() : disable_e_steppers();

  TERN_(PSU_CONTROL, PSU_OFF());

  TERN_(HAS_SUICIDE, suicide());

  #if HAS_KILL

    // Wait for kill to be released
    while (kill_state()) watchdog_refresh();

    // Wait for kill to be pressed
    while (!kill_state()) watchdog_refresh();

    void (*resetFunc)() = 0;      // Declare resetFunc() at address 0
    resetFunc();                  // Jump to address 0

  #else

    for (;;) watchdog_refresh();  // Wait for reset

  #endif
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters(); // 'unpause' taken care of in here
  print_job_timer.stop();

  #if ENABLED(PROBING_FANS_OFF)
    if (thermalManager.fans_paused) thermalManager.set_fans_paused(false); // put things back the way they were
  #endif

  if (IsRunning()) {
    SERIAL_ERROR_MSG(STR_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
    safe_delay(350);       // allow enough time for messages to get out before stopping
    marlin_state = MF_STOPPED;
  }
}

inline void tmc_standby_setup() {
  #if PIN_EXISTS(X_STDBY)
    SET_INPUT_PULLDOWN(X_STDBY_PIN);
  #endif
  #if PIN_EXISTS(X2_STDBY)
    SET_INPUT_PULLDOWN(X2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Y_STDBY)
    SET_INPUT_PULLDOWN(Y_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Y2_STDBY)
    SET_INPUT_PULLDOWN(Y2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z_STDBY)
    SET_INPUT_PULLDOWN(Z_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z2_STDBY)
    SET_INPUT_PULLDOWN(Z2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z3_STDBY)
    SET_INPUT_PULLDOWN(Z3_STDBY_PIN);
  #endif
  #if PIN_EXISTS(Z4_STDBY)
    SET_INPUT_PULLDOWN(Z4_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E0_STDBY)
    SET_INPUT_PULLDOWN(E0_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E1_STDBY)
    SET_INPUT_PULLDOWN(E1_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E2_STDBY)
    SET_INPUT_PULLDOWN(E2_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E3_STDBY)
    SET_INPUT_PULLDOWN(E3_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E4_STDBY)
    SET_INPUT_PULLDOWN(E4_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E5_STDBY)
    SET_INPUT_PULLDOWN(E5_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E6_STDBY)
    SET_INPUT_PULLDOWN(E6_STDBY_PIN);
  #endif
  #if PIN_EXISTS(E7_STDBY)
    SET_INPUT_PULLDOWN(E7_STDBY_PIN);
  #endif
}


static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP_ADDR4(&netmask,NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
  IP_ADDR4(&gw,GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  /* Add the network interface */    
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
  
  /* Registers the default network interface */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
  
  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);
}


/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 *    • Max7219
 */
void setup() {
    
  tmc_standby_setup();  // TMC Low Power Standby pins must be set early or they're not usable

  #if ENABLED(MARLIN_DEV_MODE)
    auto log_current_ms = [&](PGM_P const msg) {
      SERIAL_ECHO_START();
      SERIAL_CHAR('['); SERIAL_ECHO(millis()); SERIAL_ECHOPGM("] ");
      serialprintPGM(msg);
      SERIAL_EOL();
    };
    #define SETUP_LOG(M) log_current_ms(PSTR(M))
  #else
    #define SETUP_LOG(...) NOOP
  #endif
  #define SETUP_RUN(C) do{ SETUP_LOG(STRINGIFY(C)); C; }while(0)

  #if EITHER(DISABLE_DEBUG, DISABLE_JTAG)
    // Disable any hardware debug to free up pins for IO
    #if ENABLED(DISABLE_DEBUG) && defined(JTAGSWD_DISABLE)
      JTAGSWD_DISABLE();
    #elif defined(JTAG_DISABLE)
      JTAG_DISABLE();
    #else
      #error "DISABLE_(DEBUG|JTAG) is not supported for the selected MCU/Board."
    #endif
  #endif

  MYSERIAL0.begin(BAUDRATE);
  uint32_t serial_connect_timeout = millis() + 1000UL;
  while (!MYSERIAL0 && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
  #if HAS_MULTI_SERIAL
    MYSERIAL1.begin(BAUDRATE);
    serial_connect_timeout = millis() + 1000UL;
    while (!MYSERIAL1 && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
  #endif
  SERIAL_ECHO_MSG("start");

  #if BOTH(HAS_TFT_LVGL_UI, USE_WIFI_FUNCTION)
    mks_esp_wifi_init();
    WIFISERIAL.begin(WIFI_BAUDRATE);
    serial_connect_timeout = millis() + 1000UL;
    while (/*!WIFISERIAL && */PENDING(millis(), serial_connect_timeout)) { /*nada*/ }
  #endif

  SETUP_RUN(HAL_init());

  #if HAS_L64XX
    SETUP_RUN(L64xxManager.init());  // Set up SPI, init drivers
  #endif

  #if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
    OUT_WRITE(SMART_EFFECTOR_MOD_PIN, LOW);   // Put Smart Effector into NORMAL mode
  #endif

  #if HAS_FILAMENT_SENSOR
    SETUP_RUN(runout.setup());
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    SETUP_RUN(recovery.setup());
  #endif

  SETUP_RUN(setup_killpin());

  #if HAS_TMC220x
    SETUP_RUN(tmc_serial_begin());
  #endif

  SETUP_RUN(setup_powerhold());

  #if HAS_STEPPER_RESET
    SETUP_RUN(disableStepperDrivers());
  #endif

  #if HAS_TMC_SPI
    #if DISABLED(TMC_USE_SW_SPI)
      SETUP_RUN(SPI.begin());
    #endif
    SETUP_RUN(tmc_init_cs_pins());
  #endif

  #ifdef BOARD_INIT
    SETUP_LOG("BOARD_INIT");
    BOARD_INIT();
  #endif

  //elsan dis
  //SETUP_RUN(esp_wifi_init());

  // Check startup - does nothing if bootloader sets MCUSR to 0
  const byte mcu = HAL_get_reset_source();
  if (mcu & RST_POWER_ON) SERIAL_ECHOLNPGM(STR_POWERUP);
  if (mcu & RST_EXTERNAL) SERIAL_ECHOLNPGM(STR_EXTERNAL_RESET);
  if (mcu & RST_BROWN_OUT) SERIAL_ECHOLNPGM(STR_BROWNOUT_RESET);
  if (mcu & RST_WATCHDOG) SERIAL_ECHOLNPGM(STR_WATCHDOG_RESET);
  if (mcu & RST_SOFTWARE) SERIAL_ECHOLNPGM(STR_SOFTWARE_RESET);
  HAL_clear_reset_source();

  serialprintPGM(GET_TEXT(MSG_MARLIN));
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_EOL();
  #if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
    SERIAL_ECHO_MSG(
      " Last Updated: " STRING_DISTRIBUTION_DATE
      " | Author: " STRING_CONFIG_H_AUTHOR
    );
  #endif
  SERIAL_ECHO_MSG("Compiled: " __DATE__);
  SERIAL_ECHO_MSG(STR_FREE_MEMORY, freeMemory(), STR_PLANNER_BUFFER_BYTES, (int)sizeof(block_t) * (BLOCK_BUFFER_SIZE));

  // Init buzzer pin(s)
  #if USE_BEEPER
    SETUP_RUN(buzzer.init());
  #endif

  // Set up LEDs early
  #if HAS_COLOR_LEDS
    SETUP_RUN(leds.setup());
  #endif

  #if ENABLED(NEOPIXEL2_SEPARATE)
    SETUP_RUN(leds2.setup());
  #endif

  #if ENABLED(USE_CONTROLLER_FAN)     // Set up fan controller to initialize also the default configurations.
    SETUP_RUN(controllerFan.setup());
  #endif

  // UI must be initialized before EEPROM
  // (because EEPROM code calls the UI).

  #if ENABLED(DWIN_CREALITY_LCD)
    delay(800);   // Required delay (since boot?)
    SERIAL_ECHOPGM("\nDWIN handshake ");
    if (DWIN_Handshake()) SERIAL_ECHOLNPGM("ok."); else SERIAL_ECHOLNPGM("error.");
    DWIN_Frame_SetDir(1); // Orientation 90°
    DWIN_UpdateLCD();     // Show bootscreen (first image)
  #else
  //elsan dis
    SETUP_RUN(ui.init());
    #if HAS_WIRED_LCD && ENABLED(SHOW_BOOTSCREEN)
      SETUP_RUN(ui.show_bootscreen());
    #endif
    //elsan dis
    SETUP_RUN(ui.reset_status());     // Load welcome message early. (Retained if no errors exist.)
  #endif

  #if BOTH(SDSUPPORT, SDCARD_EEPROM_EMULATION)
    SETUP_RUN(card.mount());          // Mount media with settings before first_load
  #endif
  
  SETUP_RUN(settings.first_load());   // Load data from EEPROM if available (or use defaults)
                                      // This also updates variables in the planner, elsewhere

  #if HAS_TOUCH_XPT2046
    SETUP_RUN(touch.init());
  #endif

  TERN_(HAS_M206_COMMAND, current_position += home_offset); // Init current position based on home_offset

  sync_plan_position();               // Vital to init stepper/planner equivalent for current_position

  SETUP_RUN(thermalManager.init());   // Initialize temperature loop

  SETUP_RUN(print_job_timer.init());  // Initial setup of print job timer

  SETUP_RUN(endstops.init());         // Init endstops and pullups

  SETUP_RUN(stepper.init());          // Init stepper. This enables interrupts!

  //#define HAS_SERVOS 1 //elsan
  #if HAS_SERVOS
    SETUP_RUN(servo_init());
  #endif

  #if HAS_Z_SERVO_PROBE
    SETUP_RUN(probe.servo_probe_init());
  #endif

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CUTTER
    SETUP_RUN(cutter.init());
  #endif

  #if ENABLED(COOLANT_MIST)
    OUT_WRITE(COOLANT_MIST_PIN, COOLANT_MIST_INVERT);   // Init Mist Coolant OFF
  #endif
  #if ENABLED(COOLANT_FLOOD)
    OUT_WRITE(COOLANT_FLOOD_PIN, COOLANT_FLOOD_INVERT); // Init Flood Coolant OFF
  #endif

  #if HAS_BED_PROBE
    SETUP_RUN(endstops.enable_z_probe(false));
  #endif

  #if HAS_STEPPER_RESET
    SETUP_RUN(enableStepperDrivers());
  #endif

  #if HAS_MOTOR_CURRENT_I2C
    SETUP_RUN(digipot_i2c.init());
  #endif

  #if ENABLED(HAS_MOTOR_CURRENT_DAC)
    SETUP_RUN(stepper_dac.init());
  #endif

  #if EITHER(Z_PROBE_SLED, SOLENOID_PROBE) && HAS_SOLENOID_1
    OUT_WRITE(SOL1_PIN, LOW); // OFF
  #endif

  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // OFF
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // OFF
  #endif

  #if ENABLED(CASE_LIGHT_ENABLE)
    #if DISABLED(CASE_LIGHT_USE_NEOPIXEL)
      if (PWM_PIN(CASE_LIGHT_PIN)) SET_PWM(CASE_LIGHT_PIN); else SET_OUTPUT(CASE_LIGHT_PIN);
    #endif
    SETUP_RUN(caselight.update_brightness());
  #endif

  #if ENABLED(MK2_MULTIPLEXER)
    SETUP_LOG("MK2_MULTIPLEXER");
    SET_OUTPUT(E_MUX0_PIN);
    SET_OUTPUT(E_MUX1_PIN);
    SET_OUTPUT(E_MUX2_PIN);
  #endif

  #if HAS_FANMUX
    SETUP_RUN(fanmux_init());
  #endif

  #if ENABLED(MIXING_EXTRUDER)
    SETUP_RUN(mixer.init());
  #endif

  #if ENABLED(BLTOUCH)
    SETUP_RUN(bltouch.init(/*set_voltage=*/true));
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    SETUP_RUN(I2CPEM.init());
  #endif

  #if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
    SETUP_LOG("i2c...");
    i2c.onReceive(i2c_on_receive);
    i2c.onRequest(i2c_on_request);
  #endif

  #if DO_SWITCH_EXTRUDER
    SETUP_RUN(move_extruder_servo(0));  // Initialize extruder servo
  #endif

  #if ENABLED(SWITCHING_NOZZLE)
    SETUP_LOG("SWITCHING_NOZZLE");
    // Initialize nozzle servo(s)
    #if SWITCHING_NOZZLE_TWO_SERVOS
      lower_nozzle(0);
      raise_nozzle(1);
    #else
      move_nozzle_servo(0);
    #endif
  #endif

  #if ENABLED(MAGNETIC_PARKING_EXTRUDER)
    SETUP_RUN(mpe_settings_init());
  #endif

  #if ENABLED(PARKING_EXTRUDER)
    SETUP_RUN(pe_solenoid_init());
  #endif

  #if ENABLED(SWITCHING_TOOLHEAD)
    SETUP_RUN(swt_init());
  #endif

  #if ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    SETUP_RUN(est_init());
  #endif

  #if ENABLED(USE_WATCHDOG)
    SETUP_RUN(watchdog_init());       // Reinit watchdog after HAL_get_reset_source call
  #endif

  #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
    SETUP_RUN(closedloop.init());
  #endif

  #ifdef STARTUP_COMMANDS
    SETUP_LOG("STARTUP_COMMANDS");
    queue.inject_P(PSTR(STARTUP_COMMANDS));
  #endif

  #if ENABLED(HOST_PROMPT_SUPPORT)
    SETUP_RUN(host_action_prompt_end());
  #endif

  #if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
    SETUP_RUN(test_tmc_connection(true, true, true, true));
  #endif

  #if ENABLED(PRUSA_MMU2)
    SETUP_RUN(mmu2.init());
  #endif

  #if ENABLED(IIC_BL24CXX_EEPROM)
    BL24CXX::init();
    const uint8_t err = BL24CXX::check();
    SERIAL_ECHO_TERNARY(err, "BL24CXX Check ", "failed", "succeeded", "!\n");
  #endif

  #if ENABLED(DWIN_CREALITY_LCD)
    Encoder_Configuration();
    HMI_Init();
    HMI_StartFrame(true);
  #endif

  #if HAS_SERVICE_INTERVALS && DISABLED(DWIN_CREALITY_LCD)
    ui.reset_status(true);  // Show service messages or keep current status
  #endif

  #if ENABLED(MAX7219_DEBUG)
    SETUP_RUN(max7219.init());
  #endif

  #if ENABLED(DIRECT_STEPPING)
    SETUP_RUN(page_manager.init());
  #endif

  #if HAS_TFT_LVGL_UI
    #if ENABLED(SDSUPPORT)
      if (!card.isMounted()) SETUP_RUN(card.mount()); // Mount SD to load graphics and fonts
    #endif
    SETUP_RUN(tft_lvgl_init());
  #endif

  #if ENABLED(PASSWORD_ON_STARTUP)
    SETUP_RUN(password.lock_machine());      // Will not proceed until correct password provided
  #endif

  marlin_state = MF_RUNNING;

  SETUP_LOG("setup() completed.");

  //Elsan
  MX_GPIO_Init(); //Moved from usb_ls.
  //MX_USART6_UART_Init();
  //Serial3.begin(115200);
  //USART1->BRR = (45 << 4) | 9;  //Elsan worked, do not touch!!!
  //USART6->BRR = (45 << 4) | 9;
  //USART6->BRR = 336;  //Elsan 250000, do not forget to set and restart ESP3D.

  //USART6->BRR = 91;
  //USART3->BRR = 91;   //Not ok for 921600.
  //USART3->BRR = 729;  //57600
  //USART3->BRR = 336;  //128000
  //USART3->BRR = 556;  //76800
  //USART3->BRR = 364;  //115200. (168000000/4)/115200.
  USART3->BRR = 45;     //921600

  //USART1->BRR = 729; //Elsan TFT (DGUS) 115200.
  //USART2->BRR = 729; //Elsan TFT (DGUS) 115200. For board v2.2
  USART2->BRR = 364;
 
   //indx = 0; // buffer empty
   //process = false;
  /*  
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; 
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  //SpiHandle.Init.Mode = SPI_MODE_MASTER;
  SpiHandle.Init.Mode = SPI_MODE_SLAVE;
  //HAL_SPI_Init(&SpiHandle);
  */

  //HAL_SPI_MspInitTEST();
  //SPI3->CR1	= 0x0003;	//CPOL=1, CPHA=1
  //SPI3->CR1	|= 1<<6;	      //SPI enabled
  //SPI.attachInterrupt();
  //SPI3->CR2 = 0x0040;
  
  /*
  // Enable the peripheral clock of GPIOB 
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

  // Configure SCK Pin connected to pin 31 of CN10 connector 
  
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  // Configure MISO Pin connected to pin 27 of CN10 connector 
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  // Configure MOSI Pin connected to pin 29 of CN7 connector 
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_12, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_12, LL_GPIO_PULL_UP);
  
  //HAL_SPI_MspInitTEST();

  // (2) Configure NVIC for SPI1 transfer complete/error interrupts 
  // Set priority for SPI1_IRQn 
  NVIC_SetPriority(SPI3_IRQn, 0);
  // Enable SPI1_IRQn           
  NVIC_EnableIRQ(SPI3_IRQn);

  // Enable the peripheral clock of GPIOB 
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  // Configure SPI1 communication 
  LL_SPI_SetBaudRatePrescaler(SPI3, LL_SPI_BAUDRATEPRESCALER_DIV64);
  LL_SPI_SetTransferDirection(SPI3,LL_SPI_FULL_DUPLEX);
  LL_SPI_SetClockPhase(SPI3, LL_SPI_PHASE_2EDGE);
  LL_SPI_SetClockPolarity(SPI3, LL_SPI_POLARITY_HIGH);
  // Reset value is LL_SPI_MSB_FIRST 
  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI3, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI3, LL_SPI_NSS_SOFT);//LL_SPI_NSS_HARD_INPUT);

  LL_SPI_SetMode(SPI3, LL_SPI_MODE_SLAVE);
  // Configure SPI1 transfer interrupts 
  // Enable RXNE  Interrupt             
  LL_SPI_EnableIT_RXNE(SPI3);
  // Enable TXE   Interrupt             
  //LL_SPI_EnableIT_TXE(SPI3);
  // Enable Error Interrupt             
  LL_SPI_EnableIT_ERR(SPI3);
  */

  //HAL_SPI_MspInitTEST();
  //NVIC_EnableIRQ(SPI3_IRQn);
  //LL_SPI_Enable(SPI3); 
  
  //SPI3->CR1	= 0x0003;
  //SPI3->CR1	|= 1<<6;	//SPI enabled
  //SPI3->CR1	|= 3<<8;  //NSS is ignored.
  //SPI3->CR1	|= 2<<8;
  //SPI3->CR1	&= ~(1<<8);	//SSI->0, slave. (inverting)
  //NVIC_EnableIRQ(SPI3_IRQn);
  //SPI3->CR1	|= 1<<6;	//SPI enabled

  //Elsan new year 2021.
  __GPIOC_CLK_ENABLE();
  __SPI3_CLK_ENABLE();
  
//static SPI_HandleTypeDef spi = { .Instance = SPI3 };
spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  //SPI_BAUDRATEPRESCALER_64 ok,32, 16,8,4,2 also ok
spi.Init.Direction = SPI_DIRECTION_2LINES;
spi.Init.CLKPhase = SPI_PHASE_2EDGE; //Both worked
spi.Init.CLKPolarity = SPI_POLARITY_LOW;  //Do not change.
spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
spi.Init.DataSize = SPI_DATASIZE_8BIT;
spi.Init.FirstBit = /*SPI_FIRSTBIT_LSB*/SPI_FIRSTBIT_MSB; //Do not change.
spi.Init.NSS = /*SPI_NSS_HARD_INPUT*/SPI_NSS_SOFT;        //Do not change.
spi.Init.TIMode = SPI_TIMODE_DISABLED;
spi.Init.Mode = SPI_MODE_SLAVE; 
HAL_SPI_Init(&spi);

GPIO_InitTypeDef GPIO_InitStruct;
GPIO_InitStruct.Pin= GPIO_PIN_11;
GPIO_InitStruct.Mode= GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull= GPIO_PULLUP;
GPIO_InitStruct.Speed= GPIO_SPEED_HIGH;
GPIO_InitStruct.Alternate= GPIO_AF6_SPI3;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

GPIO_InitStruct.Pin= GPIO_PIN_10 | GPIO_PIN_12;
GPIO_InitStruct.Mode= GPIO_MODE_AF_OD;
//GPIO_InitStruct.Mode= GPIO_MODE_AF_PP;  //Elsan addition
//GPIO_InitStruct.Speed= GPIO_SPEED_HIGH; //Elsan addition
//GPIO_InitStruct.Pull= GPIO_PULLDOWN;        //Elsan addition
GPIO_InitStruct.Alternate= GPIO_AF6_SPI3;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

  //Elsan polling worked. 
  //IRQ not worked. Continue with "Polling Mode".
  //NVIC_SetPriority(SPI3_IRQn, 0);
  //NVIC_EnableIRQ(SPI3_IRQn);
  //LL_SPI_EnableIT_RXNE(SPI3);
  strcpy(output,/*input*/"R");
  //strcpy(fname2,"SPI.gco");
  
  /* Initialize the LwIP stack */
    lwip_init();

  /* Configure the Network interface */
    Netif_Config();

  /* Initialize the webserver module */
    IAP_httpd_init(); 

  /* Notify user about the network interface config */
    User_notification(&gnetif); 

  //Elsan USB Host
  __GPIOE_CLK_ENABLE();
  GPIO_InitStruct.Pin= GPIO_PIN_14;
  GPIO_InitStruct.Mode= /*GPIO_MODE_OUTPUT_PP*/GPIO_MODE_AF_OD/*GPIO_MODE_OUTPUT_OD*/;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  //HAL_GPIO_WritePin(GPIOE, /*GPIO_PIN_5*/GPIO_PIN_14, GPIO_PIN_RESET);  //Elsan USB Host: Handled in USBH_LL_DriverVBUS.  
  
  SET_INPUT_PULLUP(PD7);  //Elsan FAN1 tach (FG) input;
  SET_INPUT_PULLUP(PD4);
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->PSC = HAL_RCC_GetPCLK1Freq() / 500000;
  TIM5->CR1 = TIM_CR1_CEN;
  TIM5->CNT = -10;

  queue.inject_P(PSTR("M42 P12 M1\n M42 P13 M1")); //Make ESP_RST Output.
  queue.inject_P(PSTR("M42 P12 S255\n M42 P13 S255")); //Set ESP_RST.
}



/**
 * The main Marlin program loop
 *
 *  - Call idle() to handle all tasks between G-code commands
 *      Note that no G-codes from the queue can be executed during idle()
 *      but many G-codes can be called directly anytime like macros.
 *  - Check whether SD card auto-start is needed now.
 *  - Check whether SD print finishing is needed now.
 *  - Run one G-code command from the immediate or main command queue
 *    and open up one space. Commands in the main queue may come from sd
 *    card, host, or by direct injection. The queue will continue to fill
 *    as long as idle() or manage_inactivity() are being called.
 */

//ISR(SPI_STC_vect) // SPI interrupt routine 

void SPI3_IRQHandler(void)
{ 
    //HAL_SPI_IRQHandler(&SpiHandle);
   //byte c = SPDR; // read byte from SPI Data Register
   //LL_SPI_Disable(SPI3); 
   //SPI3->CR1	|= 3<<8;  //NSS is ignored and Slave disabled.
   //SPI3->CR1	&= ~(1<<8);	//SSI->0, slave.
   SERIAL_ECHO("SPI3_IRQHandler");
   SERIAL_ECHO("\r\n");
    /*
   byte c = SPI3->DR;
     
   if (indx < sizeof buff) {
      buff [indx++] = c; // save data in the next index in the array buff
      //if (c == '\r') //check for the end of the word
      if (c == '\n') //check for the end of the word
      process = true;
   }
   */
   //HAL_SPI_IRQHandler(&SpiHandle);
   //SPI3->CR1	&= ~(1<<8);	//SSI->0, slave.
}

/*
void SPIx_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&SpiHandle);
  SERIAL_ECHO("SPI3_IRQHandler");
  SERIAL_ECHO("\r\n");
}
*/

//void HAL_SPI_MspInitTEST(SPI_HandleTypeDef *hspi)
void HAL_SPI_MspInitTEST(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  SPIx_SCK_GPIO_CLK_ENABLE();
  SPIx_MISO_GPIO_CLK_ENABLE();
  SPIx_MOSI_GPIO_CLK_ENABLE();
  /* Enable SPI clock */
  SPIx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = SPIx_SCK_AF;
  HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
    
  /* SPI MISO GPIO pin configuration  */
  GPIO_InitStruct.Pin = SPIx_MISO_PIN;
  GPIO_InitStruct.Alternate = SPIx_MISO_AF;
  //Addition
  //GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull      = GPIO_PULLUP;
  HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
  GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
  //Addition
  //GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull      = GPIO_PULLUP;
  HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for SPI #########################################*/
  /* NVIC for SPI */
  //HAL_NVIC_SetPriority(SPIx_IRQn, 0, 1);
  //HAL_NVIC_EnableIRQ(SPIx_IRQn);
}

void usb_ls(void);

//Elsan for C files.
extern "C" void prnt_els(char * str);
void prnt_els(char * str) {
  SERIAL_ECHO(str);
  SERIAL_ECHO("\r\n");
}

extern "C" void prnt_els3(char * str);
void prnt_els3(char * str) {
  queue.inject_P(PSTR("M118 P2 USB Connected"));  
}

//Elsan for C++ files.
void prnt_els2(char * str) {
  SERIAL_ECHO(str);
  SERIAL_ECHO("\r\n");
}

UART_HandleTypeDef /*huart6*/huart3;
static void MX_USART6_UART_Init(void) //Actually for USART3
{
  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* USER CODE END USART6_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

//static void MX_GPIO_Init(void)
//{
  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOC_CLK_ENABLE();
//}

static void MX_GPIO_Init(void)
{ //Elsan this part can be put in init functions of main.
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
}


void loop() {
 unsigned char input[100]; 
 uint32_t byteswritten;
 
  do {
    
    idle();
    
    #if ENABLED(SDSUPPORT)
      //card.checkautostart();  //Elsan dis
      if (card.flag.abort_sd_printing) {abortSDPrinting(); card.flag.abort_sd_printing=false;} //Elsan change
      if (marlin_state == MF_SD_COMPLETE) finishSDPrinting(); 
    #endif

    queue.advance();
    
    endstops.event_handler();
    
    TERN_(HAS_TFT_LVGL_UI, printer_state_polling());

  } while (ENABLED(__AVR__)); // Loop forever on slower (AVR) boards
  
  //Serial6.println("Hello Serial 6");
  /*
   if (process) {
      process = false; //reset the process
      Serial.println (buff); //print the array on serial monitor
      SERIAL_ECHO(buff);
      SERIAL_ECHO("\r\n");
      indx= 0; //reset button to zero
   }
   */
   //HAL_SPI_TransmitReceive_IT(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 5);
   //HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, /*BUFFERSIZE*/5, /*5000*/10);
   //SPI3->CR1	&= ~(1<<8);	//SSI->0, slave. 
   //byte c = SPI3->DR;
   //strcpy(buff,(char*)c);
   //SERIAL_ECHO(buff);
   //SERIAL_ECHO("\r\n");
   //SPI3->CR1	|= 3<<8;  //NSS is ignored and Slave disabled.
   //while (HAL_SPI_GetState(&spi) != HAL_SPI_STATE_READY) { }
  
  //Wait until print file is closed.
  if(!isUSB_fileopen) {print_stat=0;}

  if(!isUSB_fileopen) {  //Elsan allow SPI when not printing and one file operation at a time.            
  again2: 
    if(start==1) goto again;  //V3
    //Start
    output[0]=82;//Ready message to ESP3D. Only for first start message!!!
    if(HAL_SPI_TransmitReceive(&spi, (uint8_t *)&output[0], (uint8_t*)&input[0], 1, 40)!=HAL_OK) {//send Ready message. timeou 1 bad.
      input[0]=0;
      if(start) while(HAL_SPI_TransmitReceive(&spi, (uint8_t *)&output[0], (uint8_t*)&input[0], 1, 100)!=HAL_OK);//Wait untill message is sent.
      else goto jmp;      
    }
    
    if(input[0]==0xEE) {
      //if(!start) {usb_file_open_wr();}
      //SERIAL_ECHOPGM("file opened");
      ASCIcnt=0;
      start=1;
      output[0]=0;
      goto again;
    }
        
    //Receive 1byte and stop
    output[0]=0;    
  again: 
    do {SPI3_Transfer((uint8_t *)&output[0], (uint8_t *)&input[0], 1); output[0]=input[0];} while(input[0]==0x00); 
    
    if(bufInvASCI==0) write_perm=1;//first pass, allow.
    else if((bufInvASCI>127)&&(input[0]<128)) write_perm=1;//Previous data is Inv, current one should be ASCII. 
    else if((bufInvASCI<128)&&(input[0]>127)) write_perm=1;//Previous data is ASCI, current one should be InvASCII. 
    else write_perm=0;  
    bufInvASCI=input[0];
      
    bufASCI=input[0];  //V3 data is received.
            
    if(input[0]==0xFF) {
      f_close(&MyFile);
      //SERIAL_ECHOLNPGM("closed");SERIAL_ECHO("\r");
      start=0;
      say=0;
      ftp_name_wr=1;
      fname_cnt=0;
      goto jmp;
    }
                
again5: 
    //do {SPI3_Transfer((uint8_t *)&output[0], (uint8_t *)&input[0], 1); output[0]=input[0];} while(input[0]!=0x00);
    if(bufASCI>127) bufASCI=~bufASCI;  //Convert to ASCII.
    if(write_perm) {
      if(ftp_name_wr==1){
        fname_ftp[fname_cnt]=bufASCI;
        if(bufASCI==36) { //"$"
          ftp_name_wr=0;
          fname_ftp[fname_cnt]=0; //Exclude "$"
          strcpy(fname2,fname_ftp);
          usb_file_open_wr();
        }
        fname_cnt++;
      }
      else {
        f_write(&MyFile, &bufASCI, 1, (UINT*)&byteswritten);
      }
    }
    say++;
    
jmp:     
    //Duty cycle: give more time to SPI.
    //HAL_watchdog_refresh(); //No load.
    if((say!=1024000)&&(start)) {
      goto again;
    } 
    else say=0;
  }  
  
  //LwIP Webserver
  // Read a received packet from the Ethernet buffers and send it to the lwIP for handling
    ethernetif_input(&gnetif); 
    // Handle timeouts 
    sys_check_timeouts();  

#ifdef USE_DHCP
    /* handle periodic timers for LwIP */
    DHCP_Periodic_Handle(&gnetif);
#endif

    //uint8_t str[] = "1A Hello\r\n";  
    //HAL_UART_Transmit(&huart3,str,sizeof(str),1000);
    
    x_pos=current_position.x;
    y_pos=current_position.y;
    z_pos=current_position.z;

  //Elsan Fan tach (FG) reading.   
  /* //Elsan dis for test
  cntr++; //Fan status printing only at certain frequency. 
  cntr0++;
  //Elsan more precise reading could be obtained by ISR (interrupt GPIO).
  int tach=digitalRead(PD7); 
  //int tach0=digitalRead(PD4); //Moved to Fan0 part. May cause delay here.
    
  if((cntr>=loop_speed)&&(tach==LOW)){  //Calculate and print only at certain frequency. Make sure that first tach reading is LOW. 
    for (unsigned int a=0;a<160000;a++)  { //100.000 ok
      tach=digitalRead(PD7);
      //if(!tach_first) {   
              
        //if((tach2!=tach)&&(tach==HIGH)&&(first_trig==0)&&(sec_trig==0)) { //First rising edge after LOW signal. 
        if((tach2!=tach)&&(tach)&&(!first_trig)&&(!sec_trig)) {
          //activity++; 
          //m = HAL_GetTick(); 
          us=TIM5->CNT ;
          first_trig=1;
        } 
        //else if((tach2!=tach)&&(tach==LOW)&&(first_trig==1)&&(sec_trig==0)) { //LOW after HIGH. Then allow measuring.
        else if((tach2!=tach)&&(!tach)&&(first_trig)&&(!sec_trig)) {
          //activity++; 
          //m = HAL_GetTick(); 
          //us=TIM5->CNT << 1;
          sec_trig=1;
        }
        //else if((tach2!=tach)&&(tach==HIGH)&&(sec_trig==1)) { //HIGH after LOW, measure time difference.
        else if((tach2!=tach)&&(tach)&&(sec_trig)) {
          activity++; 
          //m2 = HAL_GetTick(); 
          us2=TIM5->CNT ;
          //SERIAL_ECHOLNPAIR("Freq:",m, "Hz    RPM:",m2);
          //break;
        }
      //}
      tach2=tach; //Detect state change. LOW to HIGH or HIGH to LOW. 
      //tach_first=0;
      if(activity) break;  //Elsan one cycle is enough.
    }
    
    if(activity!=0) {
        //SERIAL_ECHOLNPGM("\n" "FAN IS RUNNING");  //Give some time for one cycle of the fan.         
        //SERIAL_ECHOLNPAIR("us:", us, "Hz    us2:", us2);
        //SERIAL_ECHOLNPAIR("FAN1 tFG:", (us2-us), "us    RPM:",45000000/(us2-us));
        SERIAL_ECHOLNPAIR("FAN1 tFG:", (us2-us), "us    RPM:",30000000/(us2-us));     //Fan is 2 pole so freq is half of measured one.
    }
    //else SERIAL_ECHOLNPGM("\n" "FAN IS STOPPED");
    cntr=0;
    activity=0;   
    tach_first=1; 
    tach2=0; 
    first_trig=0;
    sec_trig=0;
  }

  //Elsan FAN0
  int tach0=digitalRead(PD4);
  if((cntr0>=loop_speed)&&(tach0==LOW)){  //Calculate and print only at certain frequency. Make sure that first tach reading is LOW.   
    for (int a=0;a<100000;a++)  { //100.000 ok
      tach0=digitalRead(PD4);
      if((tach20!=tach0)&&(tach0==HIGH)&&(first_trig0==0)&&(sec_trig0==0)) { //First rising edge after LOW signal.             
        us0=TIM5->CNT << 1;
        first_trig0=1;
      } 
      else if((tach20!=tach0)&&(tach0==LOW)&&(first_trig0==1)&&(sec_trig0==0)) { //LOW after HIGH. Then allow measuring.          
        sec_trig0=1;
      }
      else if((tach20!=tach0)&&(tach0==HIGH)&&(sec_trig0==1)) { //HIGH after LOW, measure time difference.
        activity0++;           
        us20=TIM5->CNT << 1;          
      }      
      tach20=tach0;
      tach_first0=0;
      if(activity0>=1) break;  //Elsan one cycle is enough.
    }    
    if(activity0!=0) {        
      //SERIAL_ECHOLNPAIR("FAN0 tFG:", (us20-us0), "us    RPM:",83500000/(us20-us0));
      SERIAL_ECHOLNPAIR("FAN0 tFG:", (us20-us0), "us    RPM:",60000000/(us20-us0));     //"TIM5->CNT << 1" is multiplied by 2. Fan is 2 pole so freq is half of measured one.
    }    
    cntr0=0;
    activity0=0;   
    tach_first0=1; 
    tach20=0; 
    first_trig0=0;
    sec_trig0=0;
  } 
  */  //Elsan dis for test  

  //if(USB_check_cntr>=200) {
  if(USB_check_cntr>=50) {  
    if(!isUSB_fileopen) {
      //noUSB=1;
      usb_check();
      if(noUSB) {
        queue.inject_P(PSTR("M118 P2 No USB\n"));
        if (ScreenHandler.isOnFileListScreen())
          ScreenHandler.GotoScreen(DGUSLCD_SCREEN_NOUSB);
      }
      else queue.inject_P(PSTR("M118 P2 USB Connected\n"));
    }
    //else queue.inject_P(PSTR("M118 P2 USB Connected")); //During printing or SPI transfer no need to send.
    else USB_check_sec=0;
    USB_check_cntr=0;
  }
  USB_check_cntr++;
  
}


inline void SPI3_Transfer(uint8_t *outp, uint8_t *inp, int count) {
    while(count--) {
        while(!(SPI3->SR & SPI_SR_TXE))
            ;
        *(volatile uint8_t *)&SPI3->DR = *outp++;
        while(!(SPI3->SR & SPI_SR_RXNE))
            ;
        *inp++ = *(volatile uint8_t *)&SPI3->DR;
    }
}


