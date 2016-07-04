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

//Command/parameter string defs
const char PROGMEM cmd_version_P[] = "version";
const char PROGMEM cmd_sysid_P[] = "sysid";
const char PROGMEM cmd_heart_P[] = "heartbeat";
const char PROGMEM cmd_quit_P[] = "quit";
const char PROGMEM cmd_led_P[] = "led";
const char PROGMEM cmd_color_P[] = "color";
const char PROGMEM cmd_baud_P[] = "baud";
const char PROGMEM cmd_soft_P[] = "softbaud";
const char PROGMEM cmd_free_P[] = "free";
const char PROGMEM cmd_vars_P[] = "vars";
const char PROGMEM cmd_mode_P[] = "mode_color";
const char PROGMEM cmd_lbv_P[] = "lowcell";
const char PROGMEM cmd_lbp_P[] = "lowpct";
const char PROGMEM cmd_bright_P[] = "brightness";
const char PROGMEM cmd_anim_P[] = "animation";
const char PROGMEM cmd_freset_P[] = "factory";
const char PROGMEM cmd_minsats_P[] = "minsats";
const char PROGMEM cmd_reboot_P[] = "reboot";
const char PROGMEM cmd_help_P[] = "help";
const char PROGMEM cmd_deadband_P[] = "deadband";
const char PROGMEM cmd_lamptest_P[] = "lamptest";
//For Mavlink parameter communications
const char PROGMEM mav_led_P[] = "led_";
const char PROGMEM mav_mode_P[] = "mode_";
const char PROGMEM mav_color_P[] = "color_";

// MAVLink HeartBeat bits
#define MOTORS_ARMED 128

#define ONBOARD_PARAM_COUNT 83

#define MAVLINK_TIMEOUT  5 //Seconds of missing Mavlink before deciding link is down

#define MAV_COMP_ID_MAVPIXEL 160  //Our own component id
#define MAV_COMP_ID_VEHICLE 1     //So far as I can tell vehicles use compid 1

//Mavlink heartbeat fixed state
#define SYSTEM_MODE MAV_MODE_PREFLIGHT           ///< Booting up
#define CUSTOM_MODE 0                            ///< Custom mode, can be defined by user/adopter
//No type for a lighting controller, use next free?
#define SYSTEM_TYPE 18 //MAV_TYPE_ONBOARD_CONTROLLER; //using ONBOARD_CONTROLLER for now
#define AUTOPILOT_TYPE MAV_AUTOPILOT_INVALID     //No GCS appears to be honouring this right now..

//Desired Mavlink data stream rates
#define MAV_DATA_RATE_EXTENDED_STATUS 2
#define MAV_DATA_RATE_RC_CHANNELS 5
#define MAV_DATA_RATE_EXTRA2 5

///////////////////////////
// Global variables

// Counters and millisecond placeholders used around the code

//  Best leave heartbeat timer at one hz as it also times stream rate requests
static uint32_t hbMillis = millis();            // HeartBeat timer
#define HB_TIMER 1000                           //1hz

//Parameter send timer
static uint32_t parMillis = millis();           // Parameter timer
#define PAR_TIMER 50                            //20hz

//Led timer
static uint32_t p_led = millis();               // Blinky led startup timer
static uint32_t led_flash = 100;                //Gets changed in code to alter blink rate

// General states
byte flMode;          // Our current flight mode as defined in CheckFlightMode()
byte isArmed = 0;     // Is motors armed flag

// Host vehicle data
static float    iob_vbat_A = 0;                 // Battery A voltage in milivolt
static uint16_t iob_battery_remaining_A = 0;    // 0 to 100 <=> 0 to 1000
static uint8_t  iob_numCells = 0;               // Number of Lipo cells in the battery
static float    iob_cellVoltage = 0;            // Average per-cell battery voltage
static uint16_t iob_mode = 0;                   // Navigation mode from RC AC2 = CH5, APM = CH8
static uint16_t iob_old_mode = 0;
static uint8_t  iob_state = 0;                  // APM State (Not ready to arm, Failsafe, etc..)
static uint8_t  iob_satellites_visible = 0;     // number of satelites
static uint8_t  iob_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static unsigned int iob_hdop=0;                 
static int16_t   iob_chan1 = 1500;              // Roll
static int16_t   iob_chan2 = 1500;              // Pitch
static uint16_t  iob_throttle = 0;              // Throttle

//MAVLink session control
mavlink_system_t mavlink_system = {1,MAV_COMP_ID_MAVPIXEL,0,0};
boolean heartBeat;                      //Heartbeat enabled flag
//Host vehicle id
static uint8_t  apm_mav_type;
static int16_t  apm_mav_system = -1;    // -1 used to indicate no vehicle found 
static uint8_t  apm_mav_component;
//Mavlink parameter interface
static int16_t  m_parameter_i = ONBOARD_PARAM_COUNT;
char mavParamBuffer[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
//Mavlink heartbeat state
static uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
//Data stream rate counters
static uint8_t sr_ext_stat = 0;
static uint8_t sr_rc_chan = 0;
static uint8_t sr_extra_2 = 0;

//LED Strip vars
#ifdef LED_STRIP
uint8_t lowBattPct;
float lowBattVolt;
boolean stripAnim;
uint8_t minSats;
uint8_t deadBand;
uint8_t lampTest;
#endif

//CLI processing state
typedef struct cliConfig_s {
  char buffer[32];
  uint8_t length;
  uint8_t crlfCount;
  boolean active;
  Stream *stream;
} cliConfig_t;

