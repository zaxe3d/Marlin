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

#if NOT_TARGET(STM32F4, STM32F4xx)
  #error "Oops! Select an STM32F4 board in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "Black STM32F4VET6 supports up to 2 hotends / E-steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Black STM32F4VET6"
#endif

#define DEFAULT_MACHINE_NAME "STM32F407VGT6"

//#define I2C_EEPROM
//#define SRAM_EEPROM_EMULATION
#define FLASH_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE                0x2000  // 8KB

//
// Servos
//
//#define SERVO0_PIN                          PE13//PC6  elsan: BLTouch orange wire. Disabled as BLtouch will not be used.
//#define SERVO1_PIN                          PC7

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PC3//PE2  // X-
#define Y_DIAG_PIN                          PE6//PE1  // Y-
//#define Z_DIAG_PIN                          PE3   // Z- elsan do not open as it conflicts with Z-min pin (BLTOUCH)
//#define E0_DIAG_PIN                         PA15  // E0

//
// Limit Switches
//
//elsan
#define X_MIN_PIN                           PC3//PE2//PC13 elsan same as X_DIAG_PIN
//#define X_MAX_PIN                           PA15
#define Y_MIN_PIN                           PE6//PE1//PA5 elsan same as Y_DIAG_PIN
//#define Y_MAX_PIN                           PD12
#define Z_MIN_PIN                           PE7//PA0//PD14 PINDA
//#define Z_MAX_PIN                           PD15

//
// Z Probe must be this pins
//
//#define Z_MIN_PROBE_PIN                     PA0//PC14 Elsan: Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN overrides this.

//
// Filament Runout Sensor
//
//#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA3//PB4  // "E0-STOP"
//#endif

//
// Steppers
//
#define X_STEP_PIN                          PC2//PD1//PC4
#define X_DIR_PIN                           PC0//PD0//PA4
#define X_ENABLE_PIN                        PA0//PD3//PE7

#define Y_STEP_PIN                          PE5//PD13//PE5
#define Y_DIR_PIN                           PE4//PD12//PE2
#define Y_ENABLE_PIN                        PC13//PD14//PE6

#define Z_STEP_PIN                          PD14//PD4//PD5
#define Z_DIR_PIN                           PD15//PD15//PD3
#define Z_ENABLE_PIN                        PD10//PD2//PD6

#define E0_STEP_PIN                         PE1//PD11//PD7
#define E0_DIR_PIN                          PE0//PD7//PD0
#define E0_ENABLE_PIN                       PE3//PD10//PB9

//elsan
//#define E1_STEP_PIN                         PE0
//#define E1_DIR_PIN                          PE1
//#define E1_ENABLE_PIN                       PB8

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA5//PC0   // T0

//elsan onboard
#define TEMP_1_PIN                          PA6//PA5//PC1   // T1

#define TEMP_BED_PIN                        PB0//PA4//PC2   // TB

#define TEMP_PROBE_PIN                      PA4 //PINDA

#ifndef TEMP_CHAMBER_PIN
  //#define TEMP_CHAMBER_PIN                  PC3   // TC
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB5//PB1//PB0//PA2   // Heater0
//#define HEATER_1_PIN                        PA3   // Heater1
#define HEATER_BED_PIN                      PB4//PB0//PB1//PA1   // Hotbed

//Elsan: Normally drives mosfet. We used it to control PWM (4 wire).
#define FAN_PIN                             PB9//PE11//PE9   // Fan0
#define FAN1_PIN                            PB8//PE9//PE11  // Fan1
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
//#define LCD_PINS_D6                         //PD1  //elsan used for x motor
//#define LCD_PINS_D7                         PE8
//#define BTN_ENC                             PD9
//#define BTN_EN1                             PD4
//#define BTN_EN2                             PD13

//#define DOGLCD_CS                    LCD_PINS_D5
//#define DOGLCD_A0                    LCD_PINS_D6

//
// Onboard SD support
//
//#define SDIO_D0_PIN                         PC8
//#define SDIO_D1_PIN                         PC9
//#define SDIO_D2_PIN                         PC10
//#define SDIO_D3_PIN                         PC11
//#define SDIO_CK_PIN                         PC12
//#define SDIO_CMD_PIN                        PD2

#ifndef SDCARD_CONNECTION
  //#define SDCARD_CONNECTION              ONBOARD  //elsan disable?
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SDIO_SUPPORT                            // Use SDIO for onboard SD

  #ifndef SDIO_SUPPORT
    #define SOFTWARE_SPI                          // Use soft SPI for onboard SD
    #define SDSS                     SDIO_D3_PIN
    #define SCK_PIN                  SDIO_CK_PIN
    #define MISO_PIN                 SDIO_D0_PIN
    #define MOSI_PIN                SDIO_CMD_PIN
  #endif
#endif

//elsan
#define HAS_TMC_UART 1
/**
 * TMC2208/TMC2209 stepper drivers
 */

#if HAS_TMC_UART
  //
  // Software serial
  //

  //Elsan X/Y/Z/E_HARDWARE_SERIAL could also has been used.
  #define X_SERIAL_TX_PIN                  PB6//PD5//PB15
  #define X_SERIAL_RX_PIN                  PB7//PD6//PB15
  
  #define Y_SERIAL_TX_PIN                  PB6//PD5//PC6
  #define Y_SERIAL_RX_PIN                  PB7//PD6//PC6

  #define Z_SERIAL_TX_PIN                  PB6//PD5//PC10
  #define Z_SERIAL_RX_PIN                  PB7//PD6//PC10

  #define E0_SERIAL_TX_PIN                 PB6//PD5//PC11
  #define E0_SERIAL_RX_PIN                 PB7//PD6//PC11
  
  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE 19200
#endif