/*
 * Mavpixel Mavlink Neopixel bridge
 * (c) 2016 Nick Metcalfe
 *
 * Mavpixel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Mavpixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Mavpixel.  If not, see <http://www.gnu.org/licenses/>.
 */

//Simple command interpreter
// Emulates a portion of the Cleanflight CLI
// LED-related Cleanflight command descriptions mostly apply
// See: https://github.com/cleanflight/cleanflight/blob/master/docs/LedStrip.md

void enterCommandMode(Stream *stream) {
  out(F("\r\nMavPixel " VER " ready.\r\n#"), stream);
}

void printLed(uint8_t i, Stream *stream) {
  out(F("led "), stream);
  out(i, stream);
  out(F(" "), stream);
  printLedConfig(i, stream);
  outlf(stream);
}

void printColor(uint8_t i, Stream *stream) {
  out(F("color "), stream);
  out(i, stream);
  out(F(" "), stream);
  printColorConfig(i, stream);
  outlf(stream);
}

void printMode(uint8_t i, Stream *stream) {
  out(F("mode_color "), stream);
  out(i, stream);
  out(F(" "), stream);
  printModeConfig(i, stream);
  outlf(stream);
}

int getNumericArg(char *ptr, int maxVal, Stream *stream) {
  int i = atoi(ptr);
  if (i < 0 || i > maxVal) {
    outln(F("Range error"), stream); 
    return -1;
  }
}

boolean checkParse(boolean ok, Stream *stream) {
  if (!ok) outln(F("Parse error"), stream);
  return ok;
}

void doCommand(cliConfig_t *cli) {
    char *cp, *arg;
    int got;
    char *cmdBuffer = cli->buffer;
    Stream *stream = cli->stream;
  
    //Chop off EOL.
    while ((cp = strchr(cmdBuffer, '\r')) != 0)
        *cp = 0;
    while ((cp = strchr(cmdBuffer, '\n')) != 0)
        *cp = 0;

    //Ignore blank lines.
    got = strlen(cmdBuffer);
    if (got == 0) return;
    
    //Ignore comments.
    if (*cmdBuffer == '/') return;
    
    //Get first word
    arg = strstr(cmdBuffer, " ");
    if (arg) {
      got = arg - cmdBuffer; //length of first word
      arg += 1; //ptr to argument
    }
  
    //(v) Version info.
    if (strncmp_P(cmdBuffer, cmd_version_P, got) == 0) {
       outln(F("Version: " VER), stream);
       return;
    }

    //(sysid) Mavlink system id
    if (strncmp_P(cmdBuffer, cmd_sysid_P, got) == 0) {
      if (arg) {
        int val = atoi(arg);
        setSysid(val);
      } else {
        out(F("Sysid: "), stream);
        outln(mavlink_system.sysid, stream);
      }
      return;
    }
    
    //(heartbeat) Mavlink heartbeat
    if (strncmp_P(cmdBuffer, cmd_heart_P, got) == 0) {
      if (arg) {
        setHeartbeat((strstr(arg, "y") || strstr(arg, "Y")) > 0);
      } else {
        out(F("Heartbeat: "), stream);
        if (heartBeat) outln(F("YES"), stream); 
        else outln(F("NO"), stream);
      }
      return;
    }    

#ifdef membug
    //(f) Free RAM.
    if (strncmp_P(cmdBuffer, cmd_free_P, got) == 0) {
       out(F("Free RAM: "), stream);
       outln(freeMem(), stream);
       return;
    }
#endif

#ifdef LED_STRIP
    //(led) Configure leds
    if (strncmp_P(cmdBuffer, cmd_led_P, got) == 0) {
      if (arg) {
        int i = getNumericArg(arg, 31, stream);
        if (i < 0) return;
        //Get second word
        cp = strstr(arg, " ");
        if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          if (checkParse(parseLedStripConfig(i, cp), stream))
            writeLedConfig(i, &ledConfigs[i]);
        } else printLed(i, stream);
      } else for (int i = 0; i < ledCount; i++) printLed(i, stream);
      return;
    }

    //(color) Configure colors
    if (strncmp_P(cmdBuffer, cmd_color_P, got) == 0) {
      if (arg) {
        int i = getNumericArg(arg, 15, stream);
        if (i < 0) return;
        //Get second word
        cp = strstr(arg, " ");
        if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          if (checkParse(parseColor(i, cp), stream))
            writeColorConfig(i, colors[i]);
        } else printColor(i, stream);
      } else for (int i = 0; i < 16; i++) printColor(i, stream);
      return;
    }

    //(mode_color) Configure modes
    if (strncmp_P(cmdBuffer, cmd_mode_P, got) == 0) {
      if (arg) {
        int i = getNumericArg(arg, 20, stream);
        if (i < 0) return;
        //Get second word
        cp = strstr(arg, " ");
        if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          checkParse(parseMode(i, cp), stream);
          return;
        }
        cp = strstr(arg, ",");
        if (cp) checkParse(parseMci(arg), stream);
        else printMode(i, stream);
      } else for (int i = 0; i <= 20; i++) printMode(i, stream);
      return;
    }

    //(lowcell) Low battery cell voltage
    if (strncmp_P(cmdBuffer, cmd_lbv_P, got) == 0) {
      if (arg) {
        float val = stof(arg);
        setLowBattVolt(val);
      } else {
        out(F("Low cell: "), stream);
        out(lowBattVolt, stream);
        outln(F("v"), stream);
      }
      return;
    }

    //(lowpct) Low battery percentage
    if (strncmp_P(cmdBuffer, cmd_lbp_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 100, stream);
        if (val < 0) return;
        setLowBattPct(val);
      } else {
        out(F("Low pct: "), stream);
        out(lowBattPct, stream);
        outln(F("%"), stream);
      }
      return;
    }

    //(minsats) Minimum visible satellites
    if (strncmp_P(cmdBuffer, cmd_minsats_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 100, stream);
        if (val < 0) return;
        setMinSats(val);
      } else {
        out(F("Min sats: "), stream);
        outln(minSats, stream);
      }
      return;
    }

    //(brightness) LED strip brightness
    if (strncmp_P(cmdBuffer, cmd_bright_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 100, stream);
        if (val < 0) return;
        setBrightPct(val);
      } else {
        out(F("Brightness: "), stream);
        out((uint8_t)((float)readEEPROM(STRIP_BRIGHT) / 2.55f), stream);
        outln(F("%"), stream);
      }
      return;
    }

    //(deadband) stck movement deadband
    if (strncmp_P(cmdBuffer, cmd_deadband_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 255, stream);
        if (val < 0) return;
        setDeadband(val);
      } else {
        out(F("Deadband: "), stream);
        outln(deadBand, stream);
      }
      return;
    }

#ifdef USE_LAMPTEST
    //(lamptest)Lamptest function
    if (strncmp_P(cmdBuffer, cmd_lamptest_P, got) == 0) {
      if (arg) {
        lampTest = (strstr(arg, "y") || strstr(arg, "Y"));
      } else {
        out(F("Lamptest: "), stream);
        if (lampTest) outln(F("YES"), stream); 
        else outln(F("NO"), stream);
      }
      return;
    }    
#endif

#ifdef USE_LED_ANIMATION
    //(animation) LED strip disarmed animation
    if (strncmp_P(cmdBuffer, cmd_anim_P, got) == 0) {
      if (arg) {
        setStripAnim((strstr(arg, "y") || strstr(arg, "Y")) > 0);
      } else {
        out(F("Animation: "), stream);
        if (stripAnim) outln(F("YES"), stream); 
        else outln(F("NO"), stream);
      }
      return;
    }    
#endif

#endif

    //(factory) Factory Reset.
    if (strncmp_P(cmdBuffer, cmd_freset_P, got) == 0) {
      if (got == 7) { 
       // Factory reset request flag 
       writeEEPROM(FACTORY_RESET, 1);
       outln(F("Please reset Mavpixel"), stream);
      }
      return;
    }

    //(reboot) Reset.
    if (strncmp_P(cmdBuffer, cmd_reboot_P, got) == 0) {
      if (got == 6) softwareReboot();
    }

    //(baud) Configure Mavlink baud rate
    if (strncmp_P(cmdBuffer, cmd_baud_P, got) == 0) {
      if (arg) {
        uint32_t val = atol(arg);
        if (val > 0) setBaud(val);
      } else {
        out(F("Baud: "), stream);
        outln((uint32_t)readEP16(MAVLINK_BAUD) * 10, stream);
      }
      return;
    }

    //(softbaud) Configure SoftwareSerial baud rate
    if (strncmp_P(cmdBuffer, cmd_soft_P, got) == 0) {
      if (arg) {
#ifdef SOFTSER
        uint32_t val = atol(arg);
        if (val > 0) setSoftbaud(val);
#endif
      } else {
        out(F("Soft: "), stream);
#ifdef SOFTSER
        outln((uint32_t)readEP16(SOFTSER_BAUD) * 10, stream);
#else
        outln(0, stream);
#endif
      }
      return;
    }

#ifdef DUMPVARS
    //(vars) Variables info.
    if (strncmp_P(cmdBuffer, cmd_vars_P, got) == 0) {
       dumpVars(stream);
       return;
    }
#endif

    //(quit) Return to Mavlink mode
    if (strncmp_P(cmdBuffer, cmd_quit_P, got) == 0) {
      if (cliMavlink.active) {
        outln(F("Resuming Mavlink mode.."), stream);
        cliMavlink.active = false;
      } else outln(F("Mavlink active."), stream);
      return;
    }

    if (strncmp_P(cmdBuffer, cmd_help_P, got) == 0) {
      outln( F("List of commands:\r\n" 
      "version   \tMavPixel firmware version\r\n" 
      "sysid     \tMavlink system id\r\n" 
      "heartbeat \tEmit Mavlink heartbeats\r\n" 
#ifdef LED_STRIP
      "led       \tConfigure LEDs\r\n" 
      "color     \tConfigure colours\r\n" 
      "mode_color\tConfigure colors for modes\r\n"
      "brightness\tLED strip brightness\r\n" 
#ifdef USE_LED_ANIMATION
      "animation \tAnimation when disarmed\r\n" 
#endif
      "lowcell   \tLow battery cell voltage\r\n"
      "lowpct    \tLow battery percentage\r\n"
      "minsats   \tMinimum visible satellites\r\n"
      "deadband  \tStick center deadband\r\n" 
#ifdef USE_LAMPTEST
      "lamptest  \tShow test pattern\r\n" 
#endif
#endif
      "baud      \tSet serial baud rate\r\n" 
      "softbaud  \tSet software serial baud rate\r\n" 
      "factory   \tFactory reset\r\n"
      "reboot    \tReset the Mavpixel\r\n"
#ifdef DUMPVARS
      "vars      \tDump variables\r\n" 
#endif
#ifdef membug
      "free      \tFree RAM\r\n" 
#endif
      "help      \tThis list\r\n"
      "quit      \tExit CLI mode"
      ), stream);
      return;
    }

    //Command unknown
    outln( F("Unknown command"), stream);

}
