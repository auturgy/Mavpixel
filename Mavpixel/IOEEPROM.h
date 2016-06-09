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

/* *********************************************** */
// EEPROM Storage addresses

#define MAVLINK_BAUD  135   //16 bit
#define LOWBATT_PCT   137
#define LOWBATT_VOLT  138    //16 bit
#define STRIP_BRIGHT  140
#define FACTORY_RESET 141
#define STRIP_ANIM    142
#define SOFTSER_BAUD  143  //16 bit

// Internal version, check placeholders
#define CHK1 252
#define CHK2 253
#define VERS 254

#define LED_CONFIGS 256  //128 bytes (32 * ledConfig_t)
#define COLOR_CONFIGS 384 //64 bytes (16 * hsvColor_t)
#define MODE_CONFIGS 448 //126 bytes (21 modes * 6 colour indexes)

#define EEPROM_MAX_ADDR 1023 // This is maximum for atmel 328 chip





