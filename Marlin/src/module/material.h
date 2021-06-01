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
#pragma once

/**
 * module/material.h - material type etc.
 */

#include "../inc/MarlinConfig.h"

class Material {
public:

    typedef struct  {
      float ext;
      float bed;
    } custom_temp_t;

    // material type 0:PLA 1:ABS 2:PETG 3:FLEX 7:CUSTOM
    static uint8_t type;

    // hold custom material temp details here.
    static custom_temp_t custom_temp;

};

extern Material material;
