/*
///////////////////////////////////////////////////////////////////////
Mavpixel Mavlink Neopixel bridge
(c) 2016 Nick Metcalfe

Derived from: jD-IOBoard_MAVlink Driver
// Version      : v0.5-FrSky, 06-11-2013
// Author       : Jani Hirvinen, jani@j....com
// Co-Author(s) : 
//      Sandro Beningo     (MAVLink routines)
//      Mike Smith         (BetterStream and FastSerial libraries)



// If you use, modify, redistribute, Remember to share your modifications and 
// you need to include original authors along with your work !!
//
// Mavpixel Mavlink Neopixel bridge
// (c) 2016 Nick Metcalfe
//
// - Redistribution and use in source and binary forms, with or without 
//   modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this 
//  list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright notice, 
//  this list of conditions and the following disclaimer in the documentation 
//  and/or other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
//  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
//  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
//  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
//  POSSIBILITY OF SUCH DAMAGE.
//
/////////////////////////////////////////////////////////////////////////////
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

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

#define MAVLINK10     // Are we listening MAVLink 1.0 or 0.9   (0.9 is obsolete now)
#define HEARTBEAT     // HeartBeat signal
#define SOFTSER       //Use SoftwareSerial as configuration port
//#define DEBUG         //Output extra debug information 
#define membug        //Check memory usage
//#define DUMPVARS      //adds CLI command to dump mavlink variables 
#define LED_STRIP
#define USE_LED_GPS
#define USE_LED_ANIMATION


/* **********************************************/
/* ***************** INCLUDES *******************/

#define hiWord(w) ((w) >> 8)
#define loWord(w) ((w) & 0xff)

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>
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
#define CHKVER 43    // Version number to check from EEPROM

#define ledPin 13     // Heartbeat LED if any

//Main loop controls
int messageCounter;
static bool mavlink_active;
static bool cli_active;
byte ledState;  //Onboard led state

//Serial command buffer
char cmdBuffer[32];
uint8_t cmdLen = 0;

// Objects and Serial definitions
FastSerialPort0(Serial);

// Periodic timer
SimpleTimer  mavlinkTimer;

//LED strip IO Pins
#ifdef LED_STRIP
#define NEO_PIN1 14
#define NEO_PIN2 15
#define NEO_PIN3 16
#define NEO_PIN4 17
Adafruit_NeoPixel* strip[4];
#endif

#ifdef SOFTSER
//SoftwareSerial dbSerial(12,11);        // For debug porposes we are using pins 11, 12
//SoftwareSerial dbSerial(8,9);        // For debug porposes we are using pins 11, 12
AltSoftSerial dbSerial;        // AltSoftSerial always uses pins 9, 8
#define println dbSerial.println
#define print dbSerial.print
#else
#define println Serial.println
#define print Serial.print
#endif

/* **********************************************/
/* ***************** SETUP() *******************/

void setup() 
{
  boolean eeReset = false;
  // Check that EEPROM has initial settings, if not write them
  if((readEEPROM(CHK1) + readEEPROM(CHK2) != CHKVER) || readEEPROM(FACTORY_RESET)) {
    // Write factory settings on EEPROM
    eeReset = true;
    writeFactorySettings();
  }
  
  // Initialize Mavlink Serial port, speed
  Serial.begin((uint32_t)readEP16(MAVLINK_BAUD) * 10);

#ifdef SOFTSER
  // Our software serial is connected on pins D11 and D12
  dbSerial.begin((uint32_t)readEP16(SOFTSER_BAUD) * 10);                    // We don't want to too fast
#endif

  println(F("Mavpixel " VER " initialised."));
  if (eeReset) println(F("Factory Reset."));
  // setup mavlink port
  mavlink_comm_0_port = &Serial;

#ifdef LED_STRIP
  // Read led strip configs from EEPROM
  lowBattPct = readEEPROM(LOWBATT_PCT);
  lowBattVolt = readEP16(LOWBATT_VOLT) / 1000.0f;
  stripBright = readEEPROM(STRIP_BRIGHT);
  stripAnim = readEEPROM(STRIP_ANIM);
  minSats = readEEPROM(MIN_SATS);
  readStruct(LED_CONFIGS, (uint8_t*)ledConfigs, sizeof(ledConfigs));
  readColorConfigs();
  //Start the strip
  ledStripInit();
  setBrightness(stripBright);  
#endif

  // Jani's debug stuff  
#ifdef membug
  print(freeMem());
  println(F(" bytes free RAM."));
#endif

  println(F("Press <Enter> 3 times for CLI."));
  
  // Startup MAVLink timers, 50ms runs
  mavlinkTimer.Set(&OnMavlinkTimer, 50);

  // House cleaning, enable timers
  mavlinkTimer.Enable();
  
  // Enable MAV rate request
#ifndef SOFTSER
  enable_mav_request = DI;  //delayed start  
#else
  enable_mav_request = EN;  //start immediately
#endif

} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// The thing that goes around and around and around for eternity...
// MainLoop()
void loop() 
{
#ifdef HEARTBEAT
    HeartBeat();   // Update heartbeat LED on pin = ledPin (usually D13)
#endif

#ifdef LED_STRIP
    updateLedStrip();
#endif

    if(enable_mav_request == 1) { //Request rate control. 
      for(int n = 0; n < 3; n++) {
        request_mavlink_rates();   //Three times to certify it will be readed
        delay(50);
#ifdef LED_STRIP
        updateLedStrip();    //Update the strip between sends
#endif
      }
      enable_mav_request = 0;
      waitingMAVBeats = 0;
      lastMAVBeat = millis();    // Preventing error from delay sensing
      d_hbMillis = 100;        //Fast flash
    }  
    
    // Request rates again on every 10th check if mavlink is still dead.
#ifndef SOFTSER  //Active CLI on telemetry port pauses mavlink
    if(!mavlink_active && messageCounter >= 10 && !cli_active) {
#else
    if(!mavlink_active && messageCounter >= 10) {
#endif
      enable_mav_request = 1;
      messageCounter = 0;
    } 

#ifdef SOFTSER
    read_softser();
#endif    
    read_mavlink();
    mavlinkTimer.Run();
}

/* *********************************************** */
/* ******** functions used in main loop() ******** */

// Function that is called every 120ms
void OnMavlinkTimer()
{
  if(millis() < (lastMAVBeat + 3000)) {
    // General condition checks starts from here

    if(messageCounter >= 20 && mavlink_active) {
#ifdef DEBUG
      println(F("We lost MAVLink"));
#endif
      mavlink_active = 0;
      messageCounter = 0;
    }
  } else {
    waitingMAVBeats = 1;
  }
}

