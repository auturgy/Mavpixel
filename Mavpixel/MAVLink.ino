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

#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// this code was moved from libraries/GCS_MAVLink to allow compile
// time selection of MAVLink 1.0
BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

mavlink_system_t mavlink_system = {12,160,0,0};
uint8_t system_type = 18;//MAV_TYPE_GENERIC;    //No type for a lighting controller, use next free?
//uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
#include "Mavlink_compat.h"

//#ifdef MAVLINK10
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

//#else
//#include "../GCS_MAVLink/include/mavlink/v0.9/mavlink_types.h"
//#include "../GCS_MAVLink/include/mavlink/v0.9/ardupilotmega/mavlink.h"
//#endif

void HeartBeat() {
  uint32_t timer = millis();
  if(timer - hbMillis > hbTimer) 
  {
    hbMillis = timer;
#ifdef HEARTBEAT
#ifndef SOFTSER  //Active CLI on telemetry port pauses mavlink
    if (!cli_active) {
#else
    {
#endif
      //emit heartbeat
      mavlink_message_t msg;
      mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type, system_mode, custom_mode, system_state);
    }
#endif 
#ifdef RATEREQ
    mavStreamRequest();  //Request rate control.
#endif
    messageCounter++;    //used to timeout Mavlink in main loop
  }
}

#ifdef RATEREQ
//Request data streams from flight controller if required
// periodically called from heartbeat
void mavStreamRequest() {
  //Do rate control requests
  if(enable_mav_request > 0) { //Request rate control.
    println(F(" rate request..")); 
    enable_mav_request--;
    request_mavlink_rates();   
  }
}
#endif

//Send Mavlink parameter list
// Called from main loop
void mavSendData() {
  uint32_t timer = millis();
  if(timer - parMillis > parTimer) {
    parMillis = timer;
    //send parameters one by one
    if (m_parameter_i < ONBOARD_PARAM_COUNT) {
      mavSendParameter(m_parameter_i);
      m_parameter_i++;
    }
  }
}  


#ifdef RATEREQ
void request_mavlink_rates()
{
  const int  maxStreams = 7;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
					  MAV_DATA_STREAM_EXTENDED_STATUS,
                                          MAV_DATA_STREAM_RC_CHANNELS,
					  MAV_DATA_STREAM_POSITION,
                                          MAV_DATA_STREAM_EXTRA1, 
                                          MAV_DATA_STREAM_EXTRA2,
                                          MAV_DATA_STREAM_EXTRA3};
                                          
  const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02, 0x02};

  for (int i=0; i < maxStreams; i++) {
    	  mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
					       apm_mav_system, apm_mav_component,
					       MAVStreams[i], MAVRates[i], 1);
  }
}
#endif

#ifdef SOFTSER
void read_softser(){
  while(dbSerial.available() > 0) { 
    uint8_t c = dbSerial.read();
    //Look for CLI on SoftSerial channel
    if (countCrLf(c)) return;
    if (cli_active) {
      CLIchar(c);
    }
  }
}
#endif

//Main mavlink reader
void read_mavlink(){
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  // grabing data 
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();
#ifndef SOFTSER
    //Look for CLI on mavlink channel
    if (countCrLf(c)) return;
    if (cli_active) CLIchar(c);
    else {
#endif    
      // trying to grab msg  
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
         mavlink_active = 1;
         messageCounter = 0;
         led_flash = 500;

        // handle msg  
        switch(msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
            {
#ifdef DEBUG
              println(F("MAVLink HeartBeat"));
#endif
  	      apm_mav_system    = msg.sysid;
  	      apm_mav_component = msg.compid;
              apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);
  
              //Flight mode
              iob_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
              if(iob_mode != iob_old_mode) {
                iob_old_mode = iob_mode;
                CheckFlightMode();
              }                

              //Armed flag
              if(isBit(mavlink_msg_heartbeat_get_base_mode(&msg),MOTORS_ARMED)) {
                if(isArmed == 0) CheckFlightMode();
                isArmed = 1;  
              } 
              else isArmed = 0;
 
              //System status (starting up, failsafe)
              iob_state = mavlink_msg_heartbeat_get_system_status(&msg);
            }
            break;
            
          case MAVLINK_MSG_ID_SYS_STATUS:
            {
              iob_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);
              iob_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg);
  
#ifdef LED_STRIP
              //Detect number of cells
              if (numCells == 0 && iob_vbat_A > 7.5) {
                if(iob_vbat_A>21.2) numCells = 6;
                else if(iob_vbat_A>17) numCells = 5;
                else if(iob_vbat_A>12.8) numCells = 4;
                else if(iob_vbat_A>7.5) numCells = 3;
              }
              //Calculate cell voltage
              if (numCells > 0) cellVoltage = iob_vbat_A / numCells;
#endif

            }
            break;
            
#ifndef MAVLINK10 
          case MAVLINK_MSG_ID_GPS_RAW:
            {
              iob_fix_type = mavlink_msg_gps_raw_get_fix_type(&msg);
            }
            break;
          case MAVLINK_MSG_ID_GPS_STATUS:
            {
              iob_satellites_visible = mavlink_msg_gps_status_get_satellites_visible(&msg);
            }
            break;
#else
          case MAVLINK_MSG_ID_GPS_RAW_INT:
            { 
              iob_hdop=(mavlink_msg_gps_raw_int_get_eph(&msg)/100);
              iob_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
              iob_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            }
            break;  
#endif          

          case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            {
              iob_chan1 = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
              iob_chan2 = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
            }
            break;
  
          case MAVLINK_MSG_ID_VFR_HUD:
            {
              iob_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
              if(iob_throttle > 100 && iob_throttle < 150) iob_throttle = 100; //Temporary fix for ArduPlane 2.28
              if(iob_throttle < 0 || iob_throttle > 150) iob_throttle = 0; //Temporary fix for ArduPlane 2.28
            }
            break;
            
          case MAVLINK_MSG_ID_STATUSTEXT:
            {   
#ifdef DEBUG
             println(mavlink_msg_statustext_get_severity(&msg));
             println((char*)(&_MAV_PAYLOAD(&msg)[1]));            //print directly from mavlink buffer            
#endif
            }  
            break;
          case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	    {
		// Start sending parameters
		m_parameter_i = 0;
                println("Parameter request.");
	    }
	    break;
          case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	    {
                //Send single parameter (named parameters not supported, and not required by Mission Planner)
                mavSendParameter(mavlink_msg_param_request_read_get_param_index(&msg));
                //mavlink_msg_param_request_read_get_param_id(&msg, mavParamBuffer);
                print("Parameter request for parameter ");
                println(m_parameter_i);
	    }
	    break;
          case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	    {
              //Tell QGroundControl there is no mission
              mavlink_msg_mission_count_send(MAVLINK_COMM_0,
                apm_mav_system,
                apm_mav_component,
                0);
                println("Mission request.");
	    }
	    break;
          case MAVLINK_MSG_ID_PARAM_SET:
	    {
              mavlink_param_set_t set;
	      mavlink_msg_param_set_decode(&msg, &set);
 
	      // Check if this message is for this system
	      if (set.target_system == mavlink_system.sysid && set.target_component == mavlink_system.compid)
              {
                uint8_t index = 0;
                boolean reboot = false;
                if (strncmp_P(set.param_id, cmd_bright_P, 10) == 0) {setBrightPct(set.param_value); index = 1;}
                else if (strncmp_P(set.param_id, cmd_anim_P, 9) == 0) {
#ifdef USE_LED_ANIMATION
                  setStripAnim(set.param_value); 
#endif
                  index = 2;
                }
                if (strncmp_P(set.param_id, cmd_lbv_P, 7) == 0) {setLowBattVolt(set.param_value); index = 3;}
                if (strncmp_P(set.param_id, cmd_lbp_P, 6) == 0) {setLowBattPct(set.param_value); index = 4;}
                if (strncmp_P(set.param_id, cmd_minsats_P, 7) == 0) {setMinSats(set.param_value); index = 5;}
                if (strncmp_P(set.param_id, cmd_deadband_P, 8) == 0) {setDeadband(set.param_value); index = 6;}
                if (strncmp_P(set.param_id, cmd_baud_P, 4) == 0) {setBaud(set.param_value); index = 7;}
                if (strncmp_P(set.param_id, cmd_soft_P, 8) == 0) {setSoftbaud(set.param_value); index = 8;}
                if (strncmp_P(set.param_id, cmd_lamptest_P, 8) == 0) {lampTest = set.param_value; index = 9;}
                if (strncmp_P(set.param_id, cmd_freset_P, 7) == 0) {writeEEPROM(FACTORY_RESET, 1); index = 10;}
                if (strncmp_P(set.param_id, cmd_reboot_P, 6) == 0) {reboot = true; index = 11;}





                mavSendParameter(index);
                if (reboot) softwareReboot();
              }
            }
            break;
          default:
            //Do nothing
            break;
        }
      }
#ifndef SOFTSER
    }
#endif    
    delayMicroseconds(138);
    //next one
  }
}

mavlink_param_union_t param;
//Send a single Mavlink parameter by index
void mavSendParameter(int16_t index) {
  //Basic single-value parameters
  if (index == 0) mavlinkSendParam(cmd_version_P, -1, index, mavpixelVersion);
  else if (index == 1) mavlinkSendParam(cmd_bright_P, -1, index, (uint8_t)((float)stripBright / 2.55f));
#ifdef USE_LED_ANIMATION
  else if (index == 2) mavlinkSendParam(cmd_anim_P, -1, index, stripAnim);
#else
  else if (index == 2) mavlinkSendParam(cmd_anim_P, -1, index, 0);
#endif
  else if (index == 3) mavlinkSendParam(cmd_lbv_P, -1, index, lowBattVolt);
  else if (index == 4) mavlinkSendParam(cmd_lbp_P, -1, index, lowBattPct);
  else if (index == 5) mavlinkSendParam(cmd_minsats_P, -1, index, minSats);
  else if (index == 6) mavlinkSendParam(cmd_deadband_P, -1, index, deadBand);
  else if (index == 7) mavlinkSendParam(cmd_baud_P, -1, index, (uint32_t)readEP16(MAVLINK_BAUD) * 10);
  else if (index == 8) mavlinkSendParam(cmd_soft_P, -1, index, (uint32_t)readEP16(SOFTSER_BAUD) * 10);
  else if (index == 9) mavlinkSendParam(cmd_lamptest_P, -1, index, lampTest);
  else if (index == 10) mavlinkSendParam(cmd_freset_P, -1, index, readEEPROM(FACTORY_RESET));
  else if (index == 11) mavlinkSendParam(cmd_reboot_P, -1, index, 0);
  //LEDs - sent as 4 byte XY(1):COLOR(1):FLAGS(2)
  else if (index >= 12 && index < 44) 
  {
    memcpy(&param, &ledConfigs[index - 12], 4);     
    mavlinkSendParam(mav_led_P, index - 12, index, param.param_float);
  }
  //Modes - sent as a packed 4 byte representation - limits maximum modes to 32 (5-bit)
  // Packing: All six 5-bit color indexes in 4 bytes - 
  // North&low3bitsofEast:South&high2bitsofEast:West&low3bitsofUp:Down&high2bitsofUp
  else if (index >= 44 && index < 65) {
    uint8_t color = index - 44;
    uint16_t c = readModeColor(color, 1);
    param.bytes[0] = readModeColor(color, 0) + ((c << 5) & 0b11100000);
    param.bytes[1] = readModeColor(color, 2) + ((c << 2) & 0b11100000);
    c = readModeColor(color, 4);
    param.bytes[2] = readModeColor(color, 3) + ((c << 5) & 0b11100000);
    param.bytes[3] = readModeColor(color, 5) + ((c << 2) & 0b11100000);
    mavlinkSendParam(mav_mode_P, color, index, param.param_float);
  }
  //Colour palette - sent as 4 byte Hue(2):Sat(1):Val(1) 
  else if (index >= 65 && index < 81) {
    memcpy(&param, colors[index - 65], 4);
    mavlinkSendParam(mav_color_P, index - 65, index, param.param_float);
  }
}

//Send a parameter given a name, index and value
// if provided a non-negative nameIndex, append _<index> to name and send as uint32 rather than float
void mavlinkSendParam(const prog_char name_P[], int nameIndex, int index, float value) {
     strcpy_P(mavParamBuffer, name_P);
     if (nameIndex >= 0) itoa(nameIndex, mavParamBuffer + strlen_P(name_P), 10);
     mavlink_msg_param_value_send(MAVLINK_COMM_0, mavParamBuffer,
	  value, nameIndex >= 0 ? MAVLINK_TYPE_UINT32_T : MAVLINK_TYPE_FLOAT,
          ONBOARD_PARAM_COUNT, index);
}


