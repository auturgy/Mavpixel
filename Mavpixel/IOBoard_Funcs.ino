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

/////////////////////////////////////////////////////////////////////////////
*/
//Mavpixel general funcion calls

//Change Mavlink baud rate
void changeBaudRate(uint32_t newBaud) {
  Serial.flush();
  Serial.begin(newBaud);
  while(Serial.available()) Serial.read();
}

//Change software serial baud rate
#ifdef SOFTSER
void changeSoftRate(uint32_t newBaud) {
  dbSerial.flush();
  dbSerial.begin(newBaud);
  while(dbSerial.available()) dbSerial.read();
}
#endif

//Synchronise software serial to help prevent corruptions
void flush() {
#ifdef SOFTSER
  dbSerial.flushOutput();                   //Finish transmitting
  while (dbSerial.isReceiving()) delay(1);  //Finish receiving
#endif
}

//Check incoming characters for 3xCRLF and start CLI (& return true) if found
// Software serial CLI waits forever.
// Mavlink CLI times out after 20 seconds and is disabled if Mavlink is detected. 
boolean countCrLf(uint8_t c, cliConfig_t *cli) {
  /* allow CLI to be started by hitting enter 3 times, if no
   heartbeat packets have been received */
  if (c == '\n') {
      cli->crlfCount++;
  } else if (c != '\r') {
      cli->crlfCount = 0;
  }
  if (cli->crlfCount > 2) {
    cli->active = true;
    cli->crlfCount = 0;
    enterCommandMode(cli->stream);
    cli->length = 0;
    cli->buffer[0] = 0;
    return true;
  }
  return false;
}

//Accumulate characters into a 32-byte command buffer
// Dispatches command to CLI on <Enter> found
// handles backspace key, provides remote echo
void CLIchar(uint8_t c, cliConfig_t *cli) {
   if (c == '\n') {
     //got a full buffer
     outlf(cli->stream);
     doCommand(cli);
     out(F("#"), cli->stream);
     cli->length = 0;
     cli->buffer[0] = 0;
   } else {
     //Add character to buffer
     if (cli->length < 31) {
       if (c == 8) {              //Backspace
         if (cli->length > 0) {
           cli->length--;
           cli->buffer[cli->length] = 0;
           out((char)c, cli->stream);      //Echo 
         }
       }
       else {
         cli->buffer[cli->length] = c;
         cli->buffer[cli->length + 1] = 0;
         cli->length++;
         out((char)c, cli->stream);      //Echo 
       }
     }
   }
}

#ifdef SOFTSER
//Read software serial data into CLI line buffer
void read_softser(){
  while(dbSerial.available() > 0) { 
    uint8_t c = dbSerial.read();
    //Look for CLI on SoftSerial channel
    if (!cliSoftser.active && countCrLf(c, &cliSoftser)) return;
    if (cliSoftser.active) {
      CLIchar(c, &cliSoftser);
    }
  }
}
#endif

// Our generic flight modes for ArduCopter & ArduPlane
void CheckFlightMode() {
  if(apm_mav_type == 2) { // ArduCopter MultiRotor or ArduCopter Heli
    if(iob_mode == 0) {flMode = STAB;}   // Stabilize
    if(iob_mode == 1) {flMode = ACRO;}   // Acrobatic
    if(iob_mode == 2) {flMode = ALTH;}   // Alt Hold
    if(iob_mode == 3) {flMode = AUTO;}   // Auto
    if(iob_mode == 4) {flMode = GUID;}   // Guided
    if(iob_mode == 5) {flMode = LOIT;}   // Loiter
    if(iob_mode == 6) {flMode = RETL;}   // Return to Launch
    if(iob_mode == 7) {flMode = CIRC;}   // Circle
    if(iob_mode == 9) {flMode = LAND;}  // Land
    if(iob_mode == 11) {flMode = DRFT;}  // Drift
    if(iob_mode == 13) {flMode = SPRT;}  // Sport
    if(iob_mode == 14) {flMode = FLIP;}  // Flip
    if(iob_mode == 15) {flMode = ATUN;}  // Autotune
    if(iob_mode == 16) {flMode = POSH;}   // Poshold
    if(iob_mode == 17) {flMode = BRAK;}  // Brake
    if(iob_mode == 18) {flMode = THRO;}  // Throw
  }
  else if(apm_mav_type == 1) { // ArduPlane
    if(iob_mode == 2 ) {flMode = STAB;}  // Stabilize
    if(iob_mode == 0) {flMode = MANU;}   // Manual
    if(iob_mode == 12) {flMode = LOIT;}  // Loiter
    if(iob_mode == 11 ) {flMode = RETL;} // Return to Launch
    if(iob_mode == 5 ) {flMode = FBWA;}  // FLY_BY_WIRE_A
    if(iob_mode == 6 ) {flMode = FBWB;}  // FLY_BY_WIRE_B
    if(iob_mode == 15) {flMode = GUID;}  // GUIDED
    if(iob_mode == 10 ){flMode = AUTO;} // AUTO
    if(iob_mode == 1) {flMode = CIRC;}   // CIRCLE
    if(iob_mode == 3) {flMode = TRAN;}   // Training
    if(iob_mode == 4) {flMode = ACRO;}   // Acro
    if(iob_mode == 7) {flMode = CRUS;}   // Cruise
    if(iob_mode == 8) {flMode = ATUN;}   // Autotune
  }
}

//Onboard blinky  
void ledFlasher() {
  if (millis() - p_led > led_flash)
  {
    // save the last time you blinked the LED 
    p_led += led_flash;
    // if the LED is off turn it on and vice-versa:
    digitalWrite(ledPin, ledState = !ledState);   
  }
}

//Lamp Test pattern
#ifdef USE_LAMPTEST
void rainbowCycle() {
  static uint16_t j = 0;
  hsvColor_t c;
  c.s = 0;
  c.v = 255;
  uint16_t i;
  if (j == 360) j == 0;
  for(i = 0; i < 32; i++) {
    c.h = (j + (i * 45)) % 359;
    setLedHsv(i, &c);
  }
  show();
  j += 10;
}
#endif


// Checking if BIT is active in PARAM, return true if it is, false if not
byte isBit(byte param, byte bitfield) {
 if((param & bitfield) == bitfield) return 1;
  else return 0;  
}

//Jump to the bootloader
void softwareReboot()
{
  delay(500);
  asm volatile (" jmp (30720)");
}

//Dump Mavlink vars
#ifdef DUMPVARS
void dumpVars(Stream *stream) {
 out(F("Sats:"), stream);
 outln(iob_satellites_visible, stream);
 out(F("Fix:"), stream);
 outln(iob_fix_type, stream);
 out(F("Hdop:"), stream);
 outln(iob_hdop, stream);
 out(F("Modes:"), stream);
 outln(iob_mode, stream);
 out(F("Armed:"), stream);
 outln(isArmed, stream);
 out(F("Thr:"), stream);
 outln(iob_throttle, stream);
 out(F("BatVolt:"), stream);
 outln(iob_vbat_A, stream);
}
#endif

//Low memory usage atof() replacement
// From: http://stackoverflow.com/questions/4392665/converting-string-to-float-without-atof-in-c
float stof(const char* s){
  float rez = 0, fact = 1;
  if (*s == '-'){
    s++;
    fact = -1;
  };
  for (int point_seen = 0; *s; s++){
    if (*s == '.' && !point_seen){
      point_seen = 1; 
      continue;
    };
    int d = *s - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0f;
      rez = rez * 10.0f + (float)d;
    } else break;
  };
  return rez * fact;
};

