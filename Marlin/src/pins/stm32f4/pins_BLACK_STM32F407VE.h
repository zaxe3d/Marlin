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
#pragma once

/**
 * STM32F407VET6 with RAMPS-like shield
 * 'Black' STM32F407VET6 board - https://www.stm32duino.com/viewtopic.php?t=485
 * Shield - https://github.com/jmz52/Hardware
 */

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Black STM32F4VET6 supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Xboard" //"Black STM32F4VET6"
#endif

#define DEFAULT_MACHINE_NAME "X3" //"STM32F407VET6"

#define I2C_EEPROM
//#define SRAM_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE                0x2000  // 8K
#define I2C_SCL_PIN                       PA8
#define I2C_SDA_PIN                       PC9

//
// Servos
//
//#define SERVO0_PIN                          PC6
//#define SERVO1_PIN                          PC7

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PC3
#define Y_DIAG_PIN                          PE6

//
// Limit Switches
//
#define X_MIN_PIN                           PE11 //PC13
//#define X_MAX_PIN                           PA15
#define Y_MIN_PIN                           -1 //PA5
#define Y_MAX_PIN                           PE9 //PD12
#define Z_MIN_PIN                           PE7 //PD14
#define Z_MAX_PIN                           PE13 //PD15

//
// Filament Runout Sensor
//
//#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA3 //"E0-STOP"
//#endif

//
// Steppers
//
#define X_STEP_PIN                          PC2 //PC4
#define X_DIR_PIN                           PC0 //PA4
#define X_ENABLE_PIN                        PA0 //PE7

#define Y_STEP_PIN                          PE5
#define Y_DIR_PIN                           PE4 //PE2
#define Y_ENABLE_PIN                        PC13 //PE6

#define Z_STEP_PIN                          PD14 //PD5
#define Z_DIR_PIN                           PD15 //PD3
#define Z_ENABLE_PIN                        PD10 //PD6

#define E0_STEP_PIN                         PE1 //PD7
#define E0_DIR_PIN                          PE0 //PD0
#define E0_ENABLE_PIN                       PE3 //PB9

//#define E1_STEP_PIN                         PE0
//#define E1_DIR_PIN                          PE1
//#define E1_ENABLE_PIN                       PB8

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA5 //PC0   // T0
#define TEMP_1_PIN                          PA6 //PC1   // T1
#define TEMP_BED_PIN                        PB0 //PC2   // TB

#define TEMP_PROBE_PIN                      PA4 //PINDA

#ifndef TEMP_CHAMBER_PIN
  #define TEMP_CHAMBER_PIN                  PB1 //PC3   // TC
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB5 //PA2   // Heater0
//#define HEATER_1_PIN                        PA3   // Heater1
#define HEATER_BED_PIN                      PB4 //PA1   // Hotbed

#define FAN_PIN                             PB8 //PE9   // Fan0
//#define FAN1_PIN                            PE11  // Fan1
//#define FAN2_PIN                            PE13  // Fan2
//#define FAN3_PIN                            PE14  // Fan3

//
// Misc. Functions
//
//#define LED_PIN                             PA6
//#define LED_PIN                           PA7
//#define KILL_PIN                            PB1

//
// LCD / Controller
//
//#define SD_DETECT_PIN                     PC5
//#define SD_DETECT_PIN                     PA8   // SDIO SD_DETECT_PIN, external SDIO card reader only

//#define BEEPER_PIN                          PD10
//#define LCD_PINS_RS                         PE15
//#define LCD_PINS_ENABLE                     PD8
//#define LCD_PINS_D4                         PE10
//#define LCD_PINS_D5                         PE12
//#define LCD_PINS_D6                         PD1
//#define LCD_PINS_D7                         PE8
//#define BTN_ENC                             PD9
//#define BTN_EN1                             PD4
//#define BTN_EN2                             PD13

//#define DOGLCD_CS                    LCD_PINS_D5
//#define DOGLCD_A0                    LCD_PINS_D6

#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define BTN_ENC_EN                 LCD_PINS_D7  // Detect the presence of the encoder
#endif

//
// Onboard SD support
//
#ifndef SDCARD_CONNECTION
  //#define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SDIO_SUPPORT                            // Use SDIO for onboard SD
  #if DISABLED(SDIO_SUPPORT)
    #define SOFTWARE_SPI                          // Use soft SPI for onboard SD
    #define SDSS                            PC11
    #define SD_SCK_PIN                      PC12
    #define SD_MISO_PIN                     PC8
    #define SD_MOSI_PIN                     PD2
  #endif
#endif

#define HAS_TMC_UART 1
/**
 * TMC2208/TMC2209 stepper drivers
 */

#if HAS_TMC_UART
  //
  // Software serial
  //

  //Elsan X/Y/Z/E_HARDWARE_SERIAL could also has been used.
  #define X_SERIAL_TX_PIN                  PB6
  #define X_SERIAL_RX_PIN                  PB7

  #define Y_SERIAL_TX_PIN                  PB6
  #define Y_SERIAL_RX_PIN                  PB7

  #define Z_SERIAL_TX_PIN                  PB6
  #define Z_SERIAL_RX_PIN                  PB7

  #define E0_SERIAL_TX_PIN                 PB6
  #define E0_SERIAL_RX_PIN                 PB7

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE 19200
#endif