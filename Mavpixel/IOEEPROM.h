/*
 * Mavpixel Mavlink Neopixel bridge
 * (c) 2016 Nick Metcalfe
 * This file is derived from jD-IOBoard_MAVlink.
 *
 * Mavpixel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Please read licensing, redistribution, modifying, authors and 
 * version numbering from main sketch file. This file contains only
 * a minimal header.
 *
 * Mavpixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Mavpixel.  If not, see <http://www.gnu.org/licenses/>.
 */

/* *********************************************** */
// EEPROM Storage addresses

#define MAVLINK_BAUD  135   //16 bit
#define LOWBATT_PCT   137
#define LOWBATT_VOLT  138    //16 bit
#define STRIP_BRIGHT  140
#define FACTORY_RESET 141
#define STRIP_ANIM    142
#define SOFTSER_BAUD  143  //16 bit
#define MIN_SATS      145
#define DEADBAND      146
#define SYS_ID        147
#define HEARTBEAT_EN  148

// Internal version, check placeholders
#define VERMAJ 252
#define VERMIN 253
#define VERS 254

#define LED_CONFIGS 256  //128 bytes (32 * ledConfig_t)
#define COLOR_CONFIGS 384 //64 bytes (16 * hsvColor_t)
#define MODE_CONFIGS 448 //126 bytes (21 modes * 6 colour indexes)

#define EEPROM_MAX_ADDR 1023 // This is maximum for atmel 328 chip





