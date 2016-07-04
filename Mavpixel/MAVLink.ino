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

mavlink_param_union_t param;
 
#include "Mavlink_compat.h"

#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

//========================
//Periodic (1hz) functions
// Called from main loop
void HeartBeat() {
  if(millis() - hbMillis > HB_TIMER) 
  {
    hbMillis += HB_TIMER;

    //Send heartbeats
#ifdef HEARTBEAT
    if (!cliMavlink.active && heartBeat) {
      //Set system_state flags
      if (apm_mav_system == -1) system_state = MAV_STATE_STANDBY;
      else system_state = MAV_STATE_ACTIVE;
      //emit heartbeat
      mavlink_msg_heartbeat_send(MAVLINK_COMM_0, SYSTEM_TYPE, AUTOPILOT_TYPE, SYSTEM_MODE, CUSTOM_MODE, system_state);
    }
#endif 

    //Send rate requests
#ifdef RATEREQ
    if (apm_mav_system != -1) { //Are we seeing heartbeats from a vehicle?
      //Are we getting EXTENDED_STATUS stream data?
      if (sr_ext_stat == 0) request_mavlink_rates(MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_RATE_EXTENDED_STATUS);
      else sr_ext_stat = 0;  //Reset counter
      //Are we getting RC_CHANNELS stream data?
      if (sr_rc_chan == 0) request_mavlink_rates(MAV_DATA_STREAM_RC_CHANNELS, MAV_DATA_RATE_RC_CHANNELS);
      else sr_rc_chan = 0;  //Reset counter
      //Are we getting EXTRA2 stream data?
      if (sr_extra_2 == 0) request_mavlink_rates(MAV_DATA_STREAM_EXTRA2, MAV_DATA_RATE_EXTRA2);
      else sr_extra_2 = 0;  //Reset counter
    }
#endif

    //Check for Mavlink lost
    if (mavlinkTimeoutCounter >= MAVLINK_TIMEOUT) {
      if(mavlink_active) {
#ifdef DEBUG
        println(F("We lost MAVLink"));
#endif
        mavlink_active = 0;
        mavlinkTimeoutCounter = 0;
        apm_mav_system = -1;
        led_flash = 100;        //Fast flash
      }
    } else mavlinkTimeoutCounter++;
  }
}

//Send Mavlink parameter list
// Called from main loop
void mavSendData() {
  if(millis() - parMillis > PAR_TIMER) {
    parMillis += PAR_TIMER;
    //send parameters one by one
    if (m_parameter_i < ONBOARD_PARAM_COUNT) {
      mavSendParameter(m_parameter_i);
      m_parameter_i++;
    }
  }
}  

#ifdef RATEREQ
//Request a data stream at a given rate from vehicle
void request_mavlink_rates(uint8_t MAVStream, uint8_t MAVRate)
{
#ifdef DEBUG
  print(F("Request stream: "));
  println(MAVStream);
#endif
  mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
       apm_mav_system, apm_mav_component,
       MAVStream, MAVRate, 1);
}
#endif

//+++++++++++++++++++
//Main mavlink reader
void read_mavlink(){
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  // grabing data 
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();
    //Look for CLI on mavlink channel
    if (!mavlink_active && !cliMavlink.active && countCrLf(c, &cliMavlink)) return;
    if (!mavlink_active && cliMavlink.active) CLIchar(c, &cliMavlink);
    else 
    {
      // trying to grab msg  
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
         mavlink_active = 1;
         mavlinkTimeoutCounter = 0;
         led_flash = 500;

        // handle msg  
        switch(msg.msgid) {
          //Heartbeat
          case MAVLINK_MSG_ID_HEARTBEAT:
            {
#ifdef DEBUG
              println(F("MAVLink HeartBeat"));
#endif
              //Ignore heartbeats from other peripherals and groundstations
              if (msg.compid != MAV_COMP_ID_VEHICLE) break;
              
              //Ignore heartbeats from ground stations that share our component id
              apm_mav_type = mavlink_msg_heartbeat_get_type(&msg);
              if (apm_mav_type == MAV_TYPE_GCS) break;
  	      
              //Save the target of the host vehicle
              apm_mav_system = msg.sysid;
  	      apm_mav_component = msg.compid;
  
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
          
          //From EXT_STAT Extended status stream (battery, GPS)
          case MAVLINK_MSG_ID_SYS_STATUS:
            {
              if (sr_ext_stat < 255) sr_ext_stat++;      //Keep track of stream rates              
              iob_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);
              iob_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg);
  
#ifdef LED_STRIP
              //Detect number of cells once on first valid battery report
              if (iob_numCells == 0 && iob_vbat_A > 7.5) {
                if(iob_vbat_A>21.2) iob_numCells = 6;
                else if(iob_vbat_A>17) iob_numCells = 5;
                else if(iob_vbat_A>12.8) iob_numCells = 4;
                else if(iob_vbat_A>7.5) iob_numCells = 3;
              }
              //Calculate cell voltage
              if (iob_numCells > 0) iob_cellVoltage = iob_vbat_A / iob_numCells;
#endif

            }
            break;
          
          //From EXT_STAT Extended status stream (battery, GPS)
          case MAVLINK_MSG_ID_GPS_RAW_INT:
            { 
              iob_hdop=(mavlink_msg_gps_raw_int_get_eph(&msg)/100);
              iob_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
              iob_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            }
            break;  

          //From RC_CHAN RC channels stream
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            {
              if (sr_rc_chan < 255) sr_rc_chan++;      //Keep track of stream rates
              iob_chan1 = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
              iob_chan2 = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
            }
            break;
          
          //From EXTRA_2 Extra 2 stream
          case MAVLINK_MSG_ID_VFR_HUD:
            {
              if (sr_extra_2 < 255) sr_extra_2++;      //Keep track of stream rates
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
              mavlink_param_request_list_t request;
	      mavlink_msg_param_request_list_decode(&msg, &request);
	      // Check if this message is for this system
              if (request.target_system == mavlink_system.sysid 
                  && (request.target_component == mavlink_system.compid || request.target_component == 0))
		// Start sending parameters
		m_parameter_i = 0;
	    }
	    break;
          case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	    {
              mavlink_param_request_read_t request;
	      mavlink_msg_param_request_read_decode(&msg, &request);
	      // Check if this message is for this system
	      if (request.target_system == mavlink_system.sysid
                  && (request.target_component == mavlink_system.compid || request.target_component == 0))
                //Send single parameter (named parameters not supported, and seemingly not required by Mission Planner)
                mavSendParameter(mavlink_msg_param_request_read_get_param_index(&msg));
                //mavlink_msg_param_request_read_get_param_id(&msg, mavParamBuffer);
	    }
	    break;
          case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	    {
              mavlink_mission_request_list_t request;
	      mavlink_msg_mission_request_list_decode(&msg, &request);
	      // Check if this message is for this system
	      if (request.target_system == mavlink_system.sysid
                  && (request.target_component == mavlink_system.compid 
                  || request.target_component == 0
                  || request.target_component == MAV_COMP_ID_MISSIONPLANNER))
                //Tell QGroundControl there is no mission
                mavlink_msg_mission_count_send(MAVLINK_COMM_0,
                  apm_mav_system,
                  apm_mav_component,
                  0);
	    }
	    break;
          case MAVLINK_MSG_ID_PARAM_SET:
	    {
              mavlink_param_set_t set;
	      mavlink_msg_param_set_decode(&msg, &set);
 
	      // Check if this message is for this system
	      if (set.target_system == mavlink_system.sysid && set.target_component == mavlink_system.compid)
              {
                mavReceiveParameter(&set);
              }
            }
            break;
          case MAVLINK_MSG_ID_PING:
            {
              // process ping requests (tgt_system and tgt_comp must be zero)
              mavlink_ping_t ping;
              mavlink_msg_ping_decode(&msg, &ping);
              if(ping.target_system == 0 && (ping.target_component == 0 || ping.target_component == mavlink_system.compid))
                mavlink_msg_ping_send(MAVLINK_COMM_0, ping.time_usec, ping.seq, msg.sysid, msg.compid);
            }
            break;
          default:
            //Do nothing
            break;
        }
      }
    }
    delayMicroseconds(138);
    //next one
  }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//Set Mavlink parameter routine - from MAVLINK_MSG_ID_PARAM_SET message 
void mavReceiveParameter(mavlink_param_set_t *set) {
    uint8_t index = 0;
    boolean reboot = false;

    //Standard parameters
    if (strncmp_P(set->param_id, cmd_sysid_P, 5) == 0) {setSysid(set->param_value); index = 1;}
    else if (strncmp_P(set->param_id, cmd_heart_P, 9) == 0) {setHeartbeat(set->param_value); index = 2;}
#ifdef LED_STRIP                
    else if (strncmp_P(set->param_id, cmd_bright_P, 10) == 0) {setBrightPct(set->param_value); index = 3;}
    else if (strncmp_P(set->param_id, cmd_anim_P, 9) == 0) {index = 4;
#ifdef USE_LED_ANIMATION
        setStripAnim(set->param_value); 
#endif
    }
    else if (strncmp_P(set->param_id, cmd_lbv_P, 7) == 0) {setLowBattVolt(set->param_value); index = 5;}
    else if (strncmp_P(set->param_id, cmd_lbp_P, 6) == 0) {setLowBattPct(set->param_value); index = 6;}
    else if (strncmp_P(set->param_id, cmd_minsats_P, 7) == 0) {setMinSats(set->param_value); index = 7;}
    else if (strncmp_P(set->param_id, cmd_deadband_P, 8) == 0) {setDeadband(set->param_value); index = 8;}
    else if (strncmp_P(set->param_id, cmd_lamptest_P, 8) == 0) {lampTest = set->param_value; index = 11;}
#else  //LED_STRIP disabled, just set the reply index 
    else if (strncmp_P(set->param_id, cmd_bright_P, 10) == 0) index = 3;
    else if (strncmp_P(set->param_id, cmd_anim_P, 9) == 0) index = 4;
    else if (strncmp_P(set->param_id, cmd_lbv_P, 7) == 0) index = 5;
    else if (strncmp_P(set->param_id, cmd_lbp_P, 6) == 0) index = 6;
    else if (strncmp_P(set->param_id, cmd_minsats_P, 7) == 0) index = 7;
    else if (strncmp_P(set->param_id, cmd_deadband_P, 8) == 0) index = 8;
    else if (strncmp_P(set->param_id, cmd_lamptest_P, 8) == 0) index = 11;
#endif
    else if (strncmp_P(set->param_id, cmd_baud_P, 4) == 0) {setBaud(set->param_value); index = 9;}
    else if (strncmp_P(set->param_id, cmd_soft_P, 8) == 0) {index = 10;
#ifdef SOFSER
        setSoftbaud(set->param_value); 
#endif
    }
    else if (strncmp_P(set->param_id, cmd_freset_P, 7) == 0) {writeEEPROM(FACTORY_RESET, 1); index = 12;}
    else if (strncmp_P(set->param_id, cmd_reboot_P, 6) == 0) {reboot = true; index = 13;}
    
    //LED parameters
    else if (strncmp_P(set->param_id, mav_led_P, 4) == 0) {
        char* arg = strstr(set->param_id, "_");
        if (arg > 0) {
            index = atoi(arg + 1);
#ifdef LED_STRIP
            memcpy(&ledConfigs[index], &set->param_value, 4);
            writeLedConfig(index, &ledConfigs[index]);
#endif
            index += 14;
        }
    }
    
    //Mode parameters
    else if (strncmp_P(set->param_id, mav_mode_P, 5) == 0) {
        char* arg = strstr(set->param_id, "_");
        if (arg > 0) {
            index = atoi(arg + 1);
#ifdef LED_STRIP
            param.param_float = set->param_value;
            writeModeColor(index, 0, param.bytes[0] & 0b00011111);
            writeModeColor(index, 1, ((param.bytes[0] & 0b11100000) >> 5) + ((param.bytes[1] & 0b11100000) >> 2));
            writeModeColor(index, 2, param.bytes[1] & 0b00011111);
            writeModeColor(index, 3, param.bytes[2] & 0b00011111);
            writeModeColor(index, 4, ((param.bytes[2] & 0b11100000) >> 5) + ((param.bytes[3] & 0b11100000) >> 2));
            writeModeColor(index, 5, param.bytes[3] & 0b00011111);
#endif
            index += 46;
        }
    }
    
    //Color parameters
    else if (strncmp_P(set->param_id, mav_color_P, 6) == 0) {
        char* arg = strstr(set->param_id, "_");
        if (arg > 0) {
            index = atoi(arg + 1);
#ifdef LED_STRIP
            memcpy(colors[index], &set->param_value, 4);
            writeColorConfig(index, colors[index]);
#endif
            index += 67;
        }
    }
    //Send acknowledgement response
    mavSendParameter(index);
    //Reboot was requested, do it now
    if (reboot) softwareReboot();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//Send a single Mavlink parameter by index
void mavSendParameter(int16_t index) {  
    //Basic single-value parameters
    if (index == 0) mavlinkSendParam(cmd_version_P, -1, index, mavpixelVersion);
    else if (index == 1) mavlinkSendParam(cmd_sysid_P, -1, index, mavlink_system.sysid);
    else if (index == 2) mavlinkSendParam(cmd_heart_P, -1, index, heartBeat);
#ifdef LED_STRIP
    else if (index == 3) mavlinkSendParam(cmd_bright_P, -1, index, (uint8_t)((float)readEEPROM(STRIP_BRIGHT) / 2.55f));
#ifdef USE_LED_ANIMATION
    else if (index == 4) mavlinkSendParam(cmd_anim_P, -1, index, stripAnim);
#else
    else if (index == 4) mavlinkSendParam(cmd_anim_P, -1, index, 0);
#endif
    else if (index == 5) mavlinkSendParam(cmd_lbv_P, -1, index, lowBattVolt);
    else if (index == 6) mavlinkSendParam(cmd_lbp_P, -1, index, lowBattPct);
    else if (index == 7) mavlinkSendParam(cmd_minsats_P, -1, index, minSats);
    else if (index == 8) mavlinkSendParam(cmd_deadband_P, -1, index, deadBand);
    else if (index == 11) mavlinkSendParam(cmd_lamptest_P, -1, index, lampTest);
#else //LED_STRIP disabled, send dummy values
    else if (index == 3) mavlinkSendParam(cmd_bright_P, -1, index, 0);
    else if (index == 4) mavlinkSendParam(cmd_anim_P, -1, index, 0);
    else if (index == 5) mavlinkSendParam(cmd_lbv_P, -1, index, 0);
    else if (index == 6) mavlinkSendParam(cmd_lbp_P, -1, index, 0);
    else if (index == 7) mavlinkSendParam(cmd_minsats_P, -1, index, 0);
    else if (index == 8) mavlinkSendParam(cmd_deadband_P, -1, index, 0);
    else if (index == 11) mavlinkSendParam(cmd_lamptest_P, -1, index, 0);
#endif
    else if (index == 9) mavlinkSendParam(cmd_baud_P, -1, index, (uint32_t)readEP16(MAVLINK_BAUD) * 10);
    else if (index == 10) mavlinkSendParam(cmd_soft_P, -1, index, (uint32_t)readEP16(SOFTSER_BAUD) * 10);
    else if (index == 12) mavlinkSendParam(cmd_freset_P, -1, index, readEEPROM(FACTORY_RESET));
    else if (index == 13) mavlinkSendParam(cmd_reboot_P, -1, index, 0);

    //LEDs - sent as 4 byte XY(1):COLOR(1):FLAGS(2)
    else if (index >= 14 && index < 46) 
    {
#ifdef LED_STRIP
        mavlinkSendParam(mav_led_P, index - 14, index, *(float*)(&ledConfigs[index - 14]));
#else  //LED_STRIP disabled, send dummy values
        mavlinkSendParam(mav_led_P, index - 14, index, 0);
#endif
    }

    //Modes - sent as a packed 4 byte representation - limits maximum modes to 32 (5-bit)
    // Packing: All six 5-bit color indexes in 4 bytes - 
    // North&low3bitsofEast:South&high2bitsofEast:West&low3bitsofUp:Down&high2bitsofUp
    else if (index >= 46 && index < 67) {
        uint8_t color = index - 46;
#ifdef LED_STRIP
        uint16_t c = readModeColor(color, 1);
        param.bytes[0] = readModeColor(color, 0) + ((c << 5) & 0b11100000);
        param.bytes[1] = readModeColor(color, 2) + ((c << 2) & 0b11100000);
        c = readModeColor(color, 4);
        param.bytes[2] = readModeColor(color, 3) + ((c << 5) & 0b11100000);
        param.bytes[3] = readModeColor(color, 5) + ((c << 2) & 0b11100000);
        mavlinkSendParam(mav_mode_P, color, index, param.param_float);
#else  //LED_STRIP disabled, send dummy values
        mavlinkSendParam(mav_mode_P, color, index, 0);
#endif
    }

    //Colour palette - sent as 4 byte Hue(2):Sat(1):Val(1) 
    else if (index >= 67 && index < 83) {
#ifdef LED_STRIP
        mavlinkSendParam(mav_color_P, index - 67, index, *(float*)(&colors[index - 67]));
#else  //LED_STRIP disabled, send dummy values
        mavlinkSendParam(mav_color_P, index - 67, index, 0);
#endif
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


