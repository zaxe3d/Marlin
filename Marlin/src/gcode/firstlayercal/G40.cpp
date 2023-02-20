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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "../../inc/MarlinConfig.h"
#include "../../core/debug_out.h"

#include "../gcode.h"

#include "../../module/stepper.h"
#include "../../module/endstops.h"


const char *V2_gcodes_body[] = {
    "G1 Z4 F1000",
    "G1 X0 Y-2 Z0.2 F3000.0",
    "G1 E6 F2000",
    "G1 X60 E9 F1000.0",
    "G1 X100 E12.5 F1000.0",
    "G1 Z2 E-6 F2100.00000",

    "G1 X10 Y200 Z0.2 F3000",
    "G1 E6 F2000"

    "G1 F1000",
    //E = extrusion_length * layer_height * extrusion_width / (PI * pow(1.75, 2) / 4)
    "G1 X210 Y200   E6.652", //200 * 0.2 * 0.4 / (pi * 1.75 ^ 2 / 4) = 6.652
    "G1 X210 Y170   E0.997", // 30 * 0.2 * 0.4 / (pi * 1.75 ^ 2 / 4) = 0.997
    "G1 X10  Y170   E6.652",
    "G1 X10  Y140   E0.997",
    "G1 X210 Y140   E6.652",
    "G1 X210 Y139.5 E0.017",
    "G1 X10  Y139.5 E6.652",
    "G1 X10  Y139   E0.017",
    "G1 X210 Y139   E6.652",
    "G1 X210 Y138.5 E0.017",
    "G1 X10  Y138.5 E6.652",
    "G1 X10  Y138   E0.017",
    "G1 X210 Y138   E6.652",
    "G1 X210 Y110   E0.931",
    "G1 X10  Y110   E6.652",
    "G1 X10  Y80    E0.997",
    "G1 X210 Y80    E6.652",
    "G1 X210 Y79.5  E0.017",
    "G1 X10  Y79.5  E6.652",
    "G1 X10  Y79    E0.017",
    "G1 X210 Y79    E6.652",
    "G1 X210 Y78.5  E0.017",
    "G1 X10  Y78.5  E6.652",
    "G1 X10  Y78    E0.017",
    "G1 X210 Y78    E6.652",
    "G1 X210 Y50    E0.931",
    "G1 X10  Y50    E6.652",
    "G1 X10  Y20    E0.997",
    "G1 X210 Y20    E6.652",

    "G1 Z2 E-6 F2100",
    "G1 X0 Y200 Z10 F3000",

    "G4",

    "M107",
    "M104 S0", // turn off temperature
    "M140 S0", // turn off heatbed
    "M84"      // disable motors
};

const size_t V2_gcodes_body_sz = sizeof(V2_gcodes_body) / sizeof(V2_gcodes_body[0]);
static size_t body_gcode_sz = -1;
char gcode_string[80];


/**
 * G40 calibration print for calibrating probe offset
 */
void GcodeSuite::G40() {
   DEBUG_ECHOLNPGM("G40");
   body_gcode_sz = V2_gcodes_body_sz;
   for (uint i = 0; i < body_gcode_sz; i++) {
     sprintf_P(gcode_string, PSTR("%s"), V2_gcodes_body[i]); 
     //gcode.process_subcommands_now_P(gcode_string);
     gcode.process_subcommands_now(gcode_string); //Elsan
   }
}
