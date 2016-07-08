/*
 * Mavpixel Mavlink Neopixel bridge
 * (c) 2016 Nick Metcalfe
 * https://github.com/prickle/Mavpixel/
 *
 * Derived from: jD-IOBoard_MAVlink Driver
 * http://github.com/jdrones/jD-IOBoard
 * Version      : v0.5-FrSky, 06-11-2013
 * Copyright (c) 2013, Jani Hirvinen, jDrones & Co.
 * All rights reserved.
 * Author       : Jani Hirvinen, jani@j....com
 * Co-Author(s) : 
 *      Sandro Beningo     (MAVLink routines)
 *      Mike Smith         (BetterStream and FastSerial libraries)
 *
 * Portions derived from Cleanflight.
 * http://github.com/cleanflight/cleanflight
 *
 * Mavpixel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * - Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this 
 *  list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *  this list of conditions and the following disclaimer in the documentation 
 *  and/or other materials provided with the distribution.
 *
 *
 * Mavpixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Mavpixel.  If not, see <http://www.gnu.org/licenses/>.
 */

 
/*
//////////////////////////////////////////////////////////////////////////
//  Description: 
// 
//  This is an Arduino sketch on how to use Mavpixel LED Driver board
//  that listens MAVLink commands and changes patterns accordingly.
//
//  If you use, redistribute this please mention original source.
//
//  This version uses four strips, maximum eight leds per strip.
//  WS2812 Led strip outputs:
//   pins A0 (1-8), A1(9-16), A2(17-24), A3(25-32)
*/
 
/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

//Compilation switches
#define MAVLINK10     // (Leave this as is!) Are we listening MAVLink 1.0 or 0.9   (0.9 is obsolete now)
#define HEARTBEAT     // HeartBeat signal
#define RATEREQ       //Send stream rate requests
#define SOFTSER       //Use SoftwareSerial as configuration port
//#define DEBUG             //Output extra debug information 
#define membug            //Check memory usage
#define DUMPVARS          //adds CLI command to dump mavlink variables 
#define LED_STRIP         //Can compile without LED controller if desired
#define USE_LED_GPS       //Include LED GPS function
#define USE_LED_ANIMATION //Include LED animation while disarmed effect
#define USE_LAMPTEST      //adds command to provide test pattern on LEDs

/* **********************************************/
/* ***************** INCLUDES *******************/

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif
#include <EEPROM.h>
#include <GCS_MAVLink.h>

#ifdef membug
#include <MemoryFree.h>
#endif

#include <AltSoftSerial.h>
#include <Adafruit_NeoPixel.h>

// Configurations
#include "IOBoard.h"
#include "color.h"
#include "ledstrip.h"
#include "IOEEPROM.h"

/* *************************************************/
/* ***************** DEFINITIONS *******************/

#define VER "2.0"   // Software version
#define CHKMAJ 2    // Major version number to check from EEPROM
#define CHKMIN 0    // Minor version number to check from EEPROM
float mavpixelVersion = CHKMAJ + (float)(CHKMIN / 10);
#define ledPin 13     // Blinky LED if any

//Main loop controls
int mavlinkTimeoutCounter;
static bool mavlink_active;
static bool cli_active;
byte ledState;  //Onboard led state

// Objects and Serial definitions
FastSerialPort0(Serial);

//LED strip IO Pins
#ifdef LED_STRIP
#define NEO_FLAGS NEO_GRB + NEO_KHZ800
#define NEO_PIN1 14
#define NEO_PIN2 15
#define NEO_PIN3 16
#define NEO_PIN4 17
Adafruit_NeoPixel* strip[4];
#endif

//Printing shortcuts
//Stream output
#define outln(data, stream) stream->println(data)
#define out(data, stream) stream->print(data)
#define outlf(stream) stream->println()
//Debug and both streams output
#ifdef SOFTSER
AltSoftSerial dbSerial;        // AltSoftSerial always uses pins 9, 8
#define dbln(data) dbSerial.println(data)
#define db(data) dbSerial.print(data)
#define bothln(data) {dbSerial.println(data); Serial.println(data);}
#define both(data) {dbSerial.print(data); Serial.print(data);}
#else
#define dbln(data) Serial.println(data)
#define db(data) Serial.print(data)
#define bothln(data) Serial.println(data)
#define both(data) Serial.print(data)
#endif

//Mavlink Serial command buffer
cliConfig_t cliMavlink;

#ifdef SOFTSER
//Software Serial command buffer
cliConfig_t cliSoftser;
#endif

/* **********************************************/
/* ***************** SETUP() *******************/

void setup() 
{
  //Check EEPROM contents and versioning first
  boolean eeReset = checkEeprom();
  
  // Initialize Mavlink Serial port, speed
  Serial.begin((uint32_t)readEP16(MAVLINK_BAUD) * 10);
  cliMavlink.stream = &Serial;

#ifdef SOFTSER
  // Our software serial is connected on pins 9 and 8
  // We don't want to too fast, max baud 38400
  dbSerial.begin((uint32_t)readEP16(SOFTSER_BAUD) * 10);                    
  cliSoftser.stream = &dbSerial;
#endif

  //printing functions ready
  bothln(F("\r\nMavpixel " VER " initialised."));
  if (eeReset) bothln(F("Factory Reset."));
  // setup mavlink port (just one)
  mavlink_comm_0_port = &Serial;

  // Read led strip configs from EEPROM
#ifdef LED_STRIP
  lowBattPct = readEEPROM(LOWBATT_PCT);
  lowBattVolt = readEP16(LOWBATT_VOLT) / 1000.0f;
  stripAnim = readEEPROM(STRIP_ANIM);
  minSats = readEEPROM(MIN_SATS);
  deadBand = readEEPROM(DEADBAND);
  mavlink_system.sysid = readEEPROM(SYS_ID);
  heartBeat = readEEPROM(HEARTBEAT_EN);
  readStruct(LED_CONFIGS, (uint8_t*)ledConfigs, sizeof(ledConfigs));
  readColorConfigs();
  //Start the strip
  ledStripInit();
  setBrightness(readEEPROM(STRIP_BRIGHT));  
#endif

  // Jani's debug stuff  
#ifdef membug
  both(freeMem());
  bothln(F(" bytes free RAM."));
#endif

  bothln(F("Press <Enter> 3 times for CLI."));
  
} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// The thing that goes around and around and around for eternity...
// MainLoop()
void loop() 
{
    //Onboard LED
    ledFlasher();    // Update blinky LED on pin = ledPin (usually D13)

    //Send Mavlink heartbeat (also sends rate requests) 
    HeartBeat();     //  Heartbeat and other timed Mavlink events

#ifdef LED_STRIP
    //Update the LED strip
    updateLedStrip();
#endif

#ifdef SOFTSER
    //Process software serial CLI
    read_softser();
#endif

    //Read and interpret Mavlink (Also does CLI over Mavlink port if SOFTSER disabled)
    read_mavlink();

    //Send any queued Mavlink messages    
    mavSendData();

}

