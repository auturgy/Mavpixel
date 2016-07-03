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
//
//   EEPROM Functions for Mavpixel.
*/

// EEPROM reader/writers
// Utilities for writing and reading from the EEPROM

//Read byte
byte readEEPROM(int address) {
  return EEPROM.read(address);
}

//Write byte
void writeEEPROM(int address, byte value) {
  EEPROM.write(address, value);
}

//Read word
void writeEP16(int address, int value) {
  EEPROM.write(address, (value >> 8) & 0xff);
  EEPROM.write(address + 1, value & 0xff);
}

//Write word
int readEP16(int address) {
  int value = EEPROM.read(address) << 8;
  value += EEPROM.read(address + 1);
  return value;
}

//Read structure
void readStruct(int address, uint8_t* pointer, uint16_t size) {
  for (int i = 0; i < size; i++) {
    *pointer++ = EEPROM.read(address + i);
  }
}

//Write structure
void writeStruct(int address, uint8_t* pointer, uint16_t size) {
  for (int i = 0; i < size; i++) {
    EEPROM.write(address + i, *pointer++);
  }
}

//Write led to ledConfigs data
void writeLedConfig(uint8_t index, ledConfig_t *led) {
  int loc = LED_CONFIGS + (index * 4);
  writeStruct(loc, (uint8_t*)led, 4);
}

//Read all colours data
void readColorConfigs() {
  for (int index = 0; index < 16; index++) {
    int loc = COLOR_CONFIGS + (index * 4);
    readStruct(loc, (uint8_t*)(colors[index]), 4);
  }
}

//Write color to colors data
void writeColorConfig(uint8_t index, hsvColor_t *color) {
  int loc = COLOR_CONFIGS + (index * 4);
  writeStruct(loc, (uint8_t*)color, 4);
}

//Read byte from modeColor data
uint8_t readModeColor(uint8_t mode, uint8_t index) {
  int loc = MODE_CONFIGS + (mode * 6) + index;
  return EEPROM.read(loc);
}

//Write byte to modeColor data
void writeModeColor(uint8_t mode, uint8_t index, uint8_t color) {
  int loc = MODE_CONFIGS + (mode * 6) + index;
  EEPROM.write(loc, color);
}

// Write our latest FACTORY settings to EEPROM
void writeFactorySettings() {
  //NOTE: Serial port may not be initialised at this point.
 
#ifdef LED_STRIP
  writeStruct(LED_CONFIGS, (uint8_t*)ledConfigs, sizeof(ledConfigs));  
  for (int index = 0; index < 16; index++)
    writeColorConfig(index, colors[index]);
  writeModeColorsDefault();
#endif

 writeEP16(MAVLINK_BAUD, 5760);  // b/10
 writeEEPROM(LOWBATT_PCT, 20);
 writeEP16(LOWBATT_VOLT, 3300); // v*1000
 writeEEPROM(STRIP_BRIGHT, 16);
 writeEEPROM(STRIP_ANIM, 0);
 writeEP16(SOFTSER_BAUD, 240);  // b/10
 writeEEPROM(MIN_SATS, 6);
 writeEEPROM(DEADBAND, 40);
 writeEEPROM(SYS_ID, 1);
 writeEEPROM(HEARTBEAT_EN, 0);

 // Write details for versioncheck to EEPROM
 writeEEPROM(VERMIN, CHKMIN);
 writeEEPROM(VERMAJ, CHKMAJ);
 writeEEPROM(VERS, CHKMAJ * 10 + CHKMIN);
 
 // Factory reset request flag 
 writeEEPROM(FACTORY_RESET, 0);
}

// Check that EEPROM has initial settings, if not write them
// We can use same or newer EEPROM settings. This allows downgrading without losing settings.
boolean checkEeprom() {
  uint8_t ver_major = readEEPROM(VERMAJ);
  uint8_t ver_minor = readEEPROM(VERMIN);
  if(ver_major != CHKMAJ ||  ver_minor < CHKMIN || readEEPROM(FACTORY_RESET)) {
/*  Not needed yet..  
    if (ver_major == CHKMAJ && ver_minor + 1 == CHKMIN) {
    //if required we can upgrade the EEPROM from previous version here
      
      return false;
    }
    else
*/
    // Write factory settings on EEPROM
    writeFactorySettings();
    return true;
  }
}

//Parameter setters

#ifdef LED_STRIP
//Brightness
void setBrightPct(uint8_t percentage) {
  stripBright = (float)percentage * 2.55f + 0.5f;
  setBrightness(stripBright);
  writeEEPROM(STRIP_BRIGHT, stripBright);
}

//Low cell voltage
void setLowBattVolt(float voltage) {
  lowBattVolt = voltage;
  writeEP16(LOWBATT_VOLT, voltage * 1000);
}

//Low battery percentage
void setLowBattPct(uint8_t percentage) {
  lowBattPct = percentage;
  writeEEPROM(LOWBATT_PCT, percentage);
}

//Minimum visible satellites
void setMinSats(uint8_t satellites) {
  minSats = satellites;
  writeEEPROM(MIN_SATS, satellites);
}

//Indicator deadband
void setDeadband(int microseconds) {
  deadBand = microseconds;
  writeEEPROM(DEADBAND, microseconds);
}

//LED strip animation
#ifdef USE_LED_ANIMATION
void setStripAnim(boolean enable) {
  stripAnim = enable;
  writeEEPROM(STRIP_ANIM, stripAnim);
}
#endif
#endif

//Mavlink baud rate
void setBaud(uint32_t baud) {
  if (baud > 115200) baud = 115200;      //sensible maximum
  writeEP16(MAVLINK_BAUD, baud / 10);
  changeBaudRate(baud);
}

//Softserial baud rate
#ifdef SOFTSER
void setSoftbaud(uint32_t baud) {
  if (baud > 38400) baud = 38400;      //maximum speed softser can handle
  writeEP16(SOFTSER_BAUD, baud / 10);
  changeSoftRate(baud);
}
#endif

//Mavlink system id
void setSysid(uint8_t sysid) {
  mavlink_system.sysid = sysid;
  writeEEPROM(SYS_ID, sysid);
}

//Heartbeat enabled
void setHeartbeat(boolean beating) {
  heartBeat = beating;
  writeEEPROM(HEARTBEAT_EN, beating);
}

