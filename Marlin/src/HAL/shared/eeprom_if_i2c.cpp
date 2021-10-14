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
 * Platform-independent Arduino functions for I2C EEPROM.
 * Enable USE_SHARED_EEPROM if not supplied by the framework.
 */

#include "../../inc/MarlinConfig.h"

I2C_HandleTypeDef hi2c1; //Elsan
#if ENABLED(I2C_EEPROM)

#include "eeprom_if.h"
#include <Wire.h>

#define ADDR_WRITE_SYS 0xAE
#define ADDR_READ_SYS  0xAF
#define BLOCK_DELAY 7 //10,8:worked. 5,6:error
#define DELAY HAL_Delay
#define REG_ENDA1       0x0005
#define REG_ENDA2       0x0007
#define REG_ENDA3       0x0009
#define ADDR_WRITE     0xA6
#define ADDR_READ      0xA7
uint8_t st25dv64k_initialised = 0;
void prnt_els2(char * str);
void prnt_els4(char * str);

void eeprom_init() { /*Wire.begin();*/ } //Elsan dis

#if ENABLED(USE_SHARED_EEPROM)

#ifndef EEPROM_WRITE_DELAY
  #define EEPROM_WRITE_DELAY    5
#endif
#ifndef EEPROM_DEVICE_ADDRESS
  #define EEPROM_DEVICE_ADDRESS  0xAE //0xA6 //0x50
#endif

static constexpr uint8_t eeprom_device_address = I2C_ADDRESS(EEPROM_DEVICE_ADDRESS);

// ------------------------
// Public functions
// ------------------------
/*
void eeprom_write_byte(uint8_t *pos, unsigned char value) {
  const unsigned eeprom_address = (unsigned)pos;

  Wire.beginTransmission(eeprom_device_address);
  //Wire.beginTransmission(EEPROM_DEVICE_ADDRESS); //Elsan test
  Wire.write(int(eeprom_address >> 8));   // MSB
  Wire.write(int(eeprom_address & 0xFF)); // LSB
  Wire.write(value);
  Wire.endTransmission();

  // wait for write cycle to complete
  // this could be done more efficiently with "acknowledge polling"
  delay(EEPROM_WRITE_DELAY);
}
*/

uint8_t st25dv64k_rd_cfg(uint16_t address) {
    uint8_t _out[2] = { address >> 8, address & 0xff };
    uint8_t data;
    //st25dv64k_lock();
    HAL_I2C_Master_Transmit(&hi2c1, ADDR_WRITE_SYS, _out, 2, HAL_MAX_DELAY);
    //Wire.beginTransmission(ADDR_WRITE_SYS);
    //Wire.write(int(address >> 8));   // MSB
    //Wire.write(int(address & 0xFF)); // LSB
    //Wire.endTransmission();
    
    HAL_I2C_Master_Receive(&hi2c1, ADDR_READ_SYS, &data, 1, HAL_MAX_DELAY);
    //Wire.requestFrom(ADDR_READ_SYS, (byte)1);
    //st25dv64k_unlock();
    return data;
    //return Wire.available() ? Wire.read() : 0xFF;
}

void st25dv64k_wr_cfg(uint16_t address, uint8_t data) {
    uint8_t _out[3] = { address >> 8, address & 0xff, data };
    //st25dv64k_lock();
    HAL_I2C_Master_Transmit(&hi2c1, ADDR_WRITE_SYS, _out, 3, HAL_MAX_DELAY);
    //Wire.beginTransmission(ADDR_WRITE_SYS);  
    //Wire.write(int(address >> 8));   // MSB
    //Wire.write(int(address & 0xFF)); // LSB
    //Wire.write(data);
    //Wire.endTransmission();
    DELAY(BLOCK_DELAY);
    //st25dv64k_unlock(); // unlock must be here because other threads cannot access eeprom while writing/waiting
}

void st25dv64k_present_pwd(uint8_t *pwd) {
    uint8_t _out[19] = { 0x09, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x09, 0, 0, 0, 0, 0, 0, 0, 0 };
    if (pwd) {
        memcpy(_out + 2, pwd, 8);
        memcpy(_out + 11, pwd, 8);
    }
    //st25dv64k_lock();
    HAL_I2C_Master_Transmit(&hi2c1, ADDR_WRITE_SYS, _out, 19, HAL_MAX_DELAY);
    //Wire.beginTransmission(ADDR_WRITE_SYS);  
    //Wire.write(_out[0]);Wire.write(_out[1]);Wire.write(_out[2]);Wire.write(_out[3]);Wire.write(_out[4]);Wire.write(_out[5]);Wire.write(_out[6]);Wire.write(_out[7]);
    //Wire.write(_out[8]);Wire.write(_out[9]);Wire.write(_out[10]);Wire.write(_out[11]);Wire.write(_out[12]);Wire.write(_out[13]);Wire.write(_out[14]);Wire.write(_out[15]);
    //Wire.write(_out[16]);Wire.write(_out[17]);Wire.write(_out[18]);
    //Wire.endTransmission();
    //st25dv64k_unlock();
}


void st25dv64k_init	(void) {
 uint8_t pwd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
     if (!st25dv64k_initialised) {
         if ((st25dv64k_rd_cfg(REG_ENDA1) != 0x7f)
             && (st25dv64k_rd_cfg(REG_ENDA2) != 0xff)
             && (st25dv64k_rd_cfg(REG_ENDA3) != 0xff)) {
             st25dv64k_present_pwd(0);
             st25dv64k_wr_cfg(REG_ENDA3, 0xFF);
             st25dv64k_wr_cfg(REG_ENDA2, 0xFF);
             st25dv64k_wr_cfg(REG_ENDA1, 0x7F);
             st25dv64k_present_pwd(pwd);
  
             st25dv64k_rd_cfg(REG_ENDA1);
             st25dv64k_rd_cfg(REG_ENDA2);
             st25dv64k_rd_cfg(REG_ENDA3);
         }
         st25dv64k_initialised = 1;
     }
 }

void st25dv64k_user_write(uint16_t address, uint8_t data) {
    uint8_t _out[3] = { address >> 8, address & 0xff, data };
    //st25dv64k_lock();
    HAL_I2C_Master_Transmit(&hi2c1, ADDR_WRITE, _out, 3, HAL_MAX_DELAY);
    //Wire.beginTransmission(ADDR_WRITE);  
    //Wire.write(int(address >> 8));   // MSB
    //Wire.write(int(address & 0xFF)); // LSB
    //Wire.write(data);
    //Wire.endTransmission();
    DELAY(BLOCK_DELAY);
    //st25dv64k_unlock(); // unlock must be here because other threads cannot access eeprom while writing/waiting
}

void eeprom_write_byte(uint8_t *pos, unsigned char value)
{
    char stre[20]; //Elsan
	uint16_t adr = (uint16_t)(int)pos;
	st25dv64k_init();
	st25dv64k_user_write(adr, value);
	//DBG("EEwr %04x %02x", adr, value);
    //stre[0]=0;//clean content
    //sprintf(stre, "%d", adr);
    //prnt_els4("write adr:"); prnt_els4(stre);
    //sprintf(stre, "%d", value);
    //prnt_els4(" write value:"); prnt_els2(stre);
}

/*
uint8_t eeprom_read_byte(uint8_t *pos) {
  const unsigned eeprom_address = (unsigned)pos;

  Wire.beginTransmission(eeprom_device_address);
  //Wire.beginTransmission(EEPROM_DEVICE_ADDRESS); //Elsan test
  Wire.write(int(eeprom_address >> 8));   // MSB
  Wire.write(int(eeprom_address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(eeprom_device_address, (byte)1);
  //Wire.requestFrom(EEPROM_DEVICE_ADDRESS, 1); //Elsan
  return Wire.available() ? Wire.read() : 0xFF;
}
*/

uint8_t st25dv64k_user_read(uint16_t address) {
    uint8_t _out[2] = { address >> 8, address & 0xff };
    uint8_t data;
    //st25dv64k_lock();
    HAL_I2C_Master_Transmit(&hi2c1, ADDR_WRITE, _out, 2, HAL_MAX_DELAY);
    //Wire.beginTransmission(ADDR_WRITE);
    //Wire.write(int(address >> 8));   // MSB
    //Wire.write(int(address & 0xFF)); // LSB
    //Wire.endTransmission();

    HAL_I2C_Master_Receive(&hi2c1, ADDR_READ, &data, 1, HAL_MAX_DELAY);
    //Wire.requestFrom(ADDR_READ, (byte)1);
    //st25dv64k_unlock();
    return data;
    //return Wire.available() ? Wire.read() : 0xFF;
}

uint8_t eeprom_read_byte(uint8_t *pos)
{
    char stre[20]; //Elsan
	uint16_t adr = (uint16_t)(int)pos;
	uint8_t data;
	st25dv64k_init();
	data = st25dv64k_user_read(adr);
	//DBG("EErd %04x %02x", adr,data);
    //stre[0]=0;//clean content
    //sprintf(stre, "%d", adr);
    //prnt_els4("read adr:"); prnt_els4(stre);
    //sprintf(stre, "%d", data);
    //prnt_els4(" read value:"); prnt_els2(stre);
	return data;
}

#endif // USE_SHARED_EEPROM
#endif // I2C_EEPROM
