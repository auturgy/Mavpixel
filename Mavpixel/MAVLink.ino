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

mavlink_system_t mavlink_system = {12,1,0,0};

#include "Mavlink_compat.h"

//#ifdef MAVLINK10
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

//#else
//#include "../GCS_MAVLink/include/mavlink/v0.9/mavlink_types.h"
//#include "../GCS_MAVLink/include/mavlink/v0.9/ardupilotmega/mavlink.h"
//#endif

static uint8_t crlf_count = 0;

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
         d_hbMillis = 500;
        // handle msg
  
        switch(msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
            {
#ifdef DEBUG
              println(F("MAVLink HeartBeat"));
#endif
              mavbeat = 1;
  	      apm_mav_system    = msg.sysid;
  	      apm_mav_component = msg.compid;
              apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);
  
              iob_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
              if(iob_mode != iob_old_mode) {
                iob_old_mode = iob_mode;
                CheckFlightMode();
              }                

              if(isBit(mavlink_msg_heartbeat_get_base_mode(&msg),MOTORS_ARMED)) {
                if(isArmedOld == 0) {
                    CheckFlightMode();
                    isArmedOld = 1;
                }    
                isArmed = 1;  
              } else {
                isArmed = 0;
                isArmedOld = 0;
              }
   
              iob_state = mavlink_msg_heartbeat_get_system_status(&msg);

              lastMAVBeat = millis();
            }
            break;
            
          case MAVLINK_MSG_ID_SYS_STATUS:
            { 
              iob_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);
              iob_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg);
  
#ifdef LED_STRIP
              if (numCells == 0 && iob_vbat_A > 7.5) {
                if(iob_vbat_A>21.2) numCells = 6;
                else if(iob_vbat_A>17) numCells = 5;
                else if(iob_vbat_A>12.8) numCells = 4;
                else if(iob_vbat_A>7.5) numCells = 3;
              }
              
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

