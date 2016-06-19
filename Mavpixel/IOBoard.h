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
//
// Mavpixel definitions
// 4/6/2016

// Some basic defualts
#define EN  1     // Enable value
#define DI  0     // Disable value
#define TRUE 1    // Like we would not know what true is
#define FALSE 0   // or this too...

// Flight mode defines
#define STAB 0
#define ACRO 1
#define ALTH 2
#define AUTO 3
#define GUID 4
#define LOIT 5
#define RETL 6
#define CIRC 7
#define LAND 8
#define DRFT 9
#define SPRT 10
#define FLIP 11
#define ATUN 12
#define POSH 13
#define BRAK 14
#define THRO 15
//APM:plane
#define MANU 16
#define FBWA 17
#define FBWB 18
#define TRAN 19
#define CRUS 20

#define MAX_MODES 20

// MAVLink HeartBeat bits
#define MOTORS_ARMED 128

///////////////////////////
// Global variables

// Counters and millisecond placeholders used around the code
static long p_hbMillis;                         // HeartBeat counter
static long c_hbMillis;
static long d_hbMillis = 500;

static float    iob_vbat_A = 0;                 // Battery A voltage in milivolt
static uint16_t iob_battery_remaining_A = 0;    // 0 to 100 <=> 0 to 1000

static uint16_t iob_mode = 0;                   // Navigation mode from RC AC2 = CH5, APM = CH8
static uint16_t iob_old_mode = 0;
static uint8_t iob_state = 0;

static uint8_t  iob_satellites_visible = 0;     // number of satelites
static uint8_t  iob_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static unsigned int iob_hdop=0;

static int16_t   iob_chan1 = 1500;              //Roll
static int16_t   iob_chan2 = 1500;              //Pitch
static uint16_t  iob_throttle = 0;               // throtle

//MAVLink session control
static boolean  mavbeat = 0;
static float    lastMAVBeat = 0;
static boolean  waitingMAVBeats = 1;
static uint8_t  apm_mav_type;
static uint8_t  apm_mav_system; 
static uint8_t  apm_mav_component;
static boolean  enable_mav_request = 0;

// General states
byte flMode;          // Our current flight mode as defined
byte isArmed = 0;     // Is motors armed flag
byte isArmedOld = 0;  // Earlier Armed status flag

#ifdef LED_STRIP
uint8_t lowBattPct;
float lowBattVolt;
uint8_t numCells = 0;
float cellVoltage = 0;
uint8_t stripBright = 0;
boolean stripAnim;
uint8_t minSats;
uint8_t deadBand;
#endif

