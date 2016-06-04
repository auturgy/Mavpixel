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
  writeStruct(COLOR_CONFIGS, (uint8_t*)colors, sizeof(colors));
  writeModeColorsDefault();
#endif

 writeEP16(MAVLINK_BAUD, 5760);  // b/10
 writeEEPROM(LOWBATT_PCT, 20);
 writeEP16(LOWBATT_VOLT, 3300); // v*1000
 writeEEPROM(STRIP_BRIGHT, 255);
 writeEEPROM(STRIP_ANIM, 0);

 // Write details for versioncheck to EEPROM
 writeEEPROM(CHK1, 22);
 writeEEPROM(CHK2, 21);
 writeEEPROM(VERS, CHKVER);
 
 // Factory reset request flag 
 writeEEPROM(FACTORY_RESET, 0);
}
