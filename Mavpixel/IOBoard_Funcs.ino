/*
///////////////////////////////////////////////////////////////////////
//
// Please read licensing, redistribution, modifying, authors and 
// version numbering from main sketch file. This file contains only
// a minimal header.
//
// Mavpixel Mavlink Neopixel bridge
// (c) 2016 Nick Metcalfe
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

//Check incoming characters for 3xCRLF and start CLI (return true) if found
// Software serial CLI waits forever.
// Mavlink CLI times out after 20 seconds and is disabled if Mavlink is detected. 
boolean countCrLf(uint8_t c) {
    /* allow CLI to be started by hitting enter 3 times, if no
     heartbeat packets have been received */
#ifndef SOFTSER
  if (mavlink_active == 0 && cli_active == 0 && millis() < 20000) {
#else
  if (cli_active == 0) {
#endif
      if (c == '\n') {
          crlf_count++;
      } else if (c != '\r') {
          crlf_count = 0;
      }
      if (crlf_count > 2) {
        cli_active = true;
        crlf_count = 0;
        enterCommandMode();
        cmdLen = 0;
        cmdBuffer[0] = 0;
        return true;
      }
  }
  return false;
}

//Accumulate characters into a 32-byte command buffer
// Dispatches command to CLI on <Enter> found
// handles backspace key, provides remote echo
void CLIchar(uint8_t c) {
   if (c == '\n') {
     //got a full buffer
     println();
     doCommand();
     print(F("#"));
     cmdLen = 0;
     cmdBuffer[0] = 0;
   } else {
     //Add character to buffer
     if (cmdLen < 31) {
       if (c == 8) {              //Backspace
         if (cmdLen > 0) {
           cmdLen--;
           cmdBuffer[cmdLen] = 0;
           print((char)c);      //Echo 
         }
       }
       else {
         cmdBuffer[cmdLen] = c;
         cmdBuffer[cmdLen + 1] = 0;
         cmdLen++;
         print((char)c);      //Echo 
       }
     }
   }
}

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
  uint32_t timer = millis();
  if (timer - p_led > led_flash)
  {
    // save the last time you blinked the LED 
    p_led += led_flash;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);   
  }
}

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
void dumpVars() {
 print(F("Sats:"));
 println(iob_satellites_visible);
 print(F("Fix:"));
 println(iob_fix_type);
 print(F("Hdop:"));
 println(iob_hdop);
 print(F("Modes:"));
 println(iob_mode);
 print(F("Armed:"));
 println(isArmed);
 print(F("Thr:"));
 println(iob_throttle);
 print(F("BatVolt:"));
 println(iob_vbat_A);
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

