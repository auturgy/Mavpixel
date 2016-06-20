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

const char PROGMEM cmd_version_P[] = "version";
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


void enterCommandMode() {
  print(F("\r\nMavPixel " VER " ready.\r\n#"));
}

void printLed(uint8_t i) {
  print(F("led "));
  print(i);
  print(F(" "));
  printLedConfig(i);
  println();
}

void printColor(uint8_t i) {
  print(F("color "));
  print(i);
  print(F(" "));
  printColorConfig(i);
  println();
}

void printMode(uint8_t i) {
  print(F("mode_color "));
  print(i);
  print(F(" "));
  printModeConfig(i);
  println();
}

int getNumericArg(char *ptr, int maxVal) {
  int i = atoi(ptr);
  if (i < 0 || i > maxVal) {
    println(F("Range error")); 
    return -1;
  }
}

boolean checkParse(boolean ok) {
  if (!ok) println(F("Parse error"));
  return ok;
}

void doCommand() {
    char *cp, *arg;
    int got;
  
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
       println(F("Version: " VER));
       return;
    }

#ifdef membug
    //(f) Free RAM.
    if (strncmp_P(cmdBuffer, cmd_free_P, got) == 0) {
          print(F("Free RAM: "));
          println(freeMem());
       return;
    }
#endif

#ifdef LED_STRIP
    //(led) Configure leds
    if (strncmp_P(cmdBuffer, cmd_led_P, got) == 0) {
      if (arg) {
        int i = getNumericArg(arg, 31);
        if (i < 0) return;
        //Get second word
        cp = strstr(arg, " ");
        if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          if (checkParse(parseLedStripConfig(i, cp)))
            writeLedConfig(i, &ledConfigs[i]);
        } else printLed(i);
      } else for (int i = 0; i < ledCount; i++) printLed(i);
      return;
    }

    //(color) Configure colors
    if (strncmp_P(cmdBuffer, cmd_color_P, got) == 0) {
      if (arg) {
        int i = getNumericArg(arg, 15);
        if (i < 0) return;
        //Get second word
        cp = strstr(arg, " ");
        if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          if (checkParse(parseColor(i, cp)))
            writeColorConfig(i, colors[i]);
        } else printColor(i);
      } else for (int i = 0; i < 16; i++) printColor(i);
      return;
    }

    //(mode_color) Configure modes
    if (strncmp_P(cmdBuffer, cmd_mode_P, got) == 0) {
      if (arg) {
        int i = getNumericArg(arg, 20);
        if (i < 0) return;
        //Get second word
        cp = strstr(arg, " ");
        if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          checkParse(parseMode(i, cp));
        }
        cp = strstr(arg, ",");
        if (cp) checkParse(parseMci(arg));
        else printMode(i);
      } else for (int i = 0; i <= 20; i++) printMode(i);
      return;
    }

    //(lowcell) Low battery cell voltage
    if (strncmp_P(cmdBuffer, cmd_lbv_P, got) == 0) {
      if (arg) {
        float val = stof(arg);
        lowBattVolt = val;
        writeEP16(LOWBATT_VOLT, val * 1000);
      } else {
        print(F("Low cell: "));
        print(lowBattVolt);
        println(F("v"));
      }
      return;
    }

    //(lowpct) Low battery percentage
    if (strncmp_P(cmdBuffer, cmd_lbp_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 100);
        if (val < 0) return;
        lowBattPct = val;
        writeEEPROM(LOWBATT_PCT, val);
      } else {
        print(F("Low pct: "));
        print(lowBattPct);
        println(F("%"));
      }
      return;
    }

    //(minsats) Minimum visible satellites
    if (strncmp_P(cmdBuffer, cmd_minsats_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 100);
        if (val < 0) return;
        minSats = val;
        writeEEPROM(MIN_SATS, val);
      } else {
        print(F("Min sats: "));
        println(minSats);
      }
      return;
    }

    //(brightness) LED strip brightness
    if (strncmp_P(cmdBuffer, cmd_bright_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 100);
        if (val < 0) return;
        stripBright = (float)val * 2.55f + 0.5f;
        setBrightness(stripBright);
        writeEEPROM(STRIP_BRIGHT, stripBright);
      } else {
        print(F("Brightness: "));
        print((uint8_t)((float)stripBright / 2.55f));
        println(F("%"));
      }
      return;
    }

    //(deadband) stck movement deadband
    if (strncmp_P(cmdBuffer, cmd_deadband_P, got) == 0) {
      if (arg) {
        int val = getNumericArg(arg, 255);
        if (val < 0) return;
        deadBand = val;
        writeEEPROM(DEADBAND, deadBand);
      } else {
        print(F("Deadband: "));
        println(deadBand);
      }
      return;
    }

#ifdef LAMPTEST
    //(lamptest)Lamptest function
    if (strncmp_P(cmdBuffer, cmd_lamptest_P, got) == 0) {
      if (arg) {
        lampTest = (strstr(arg, "y") || strstr(arg, "Y"));
      } else {
        print(F("Lamptest: "));
        if (lampTest) println(F("YES")); 
        else println(F("NO"));
      }
      return;
    }    
#endif

#ifdef USE_LED_ANIMATION
    //(animation) LED strip disarmed animation
    if (strncmp_P(cmdBuffer, cmd_anim_P, got) == 0) {
      if (arg) {
        stripAnim = (strstr(arg, "y") || strstr(arg, "Y"));
        writeEEPROM(STRIP_ANIM, stripAnim);
      } else {
        print(F("Animation: "));
        if (stripAnim) println(F("YES")); 
        else println(F("NO"));
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
       println(F("Please reset Mavpixel"));
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
        if (val > 0) {
          writeEP16(MAVLINK_BAUD, val / 10);
          changeBaudRate(val);
        }
      } else {
        print(F("Baud: "));
        println((uint32_t)readEP16(MAVLINK_BAUD) * 10);
      }
      return;
    }

    //(softbaud) Configure SoftwareSerial baud rate
    if (strncmp_P(cmdBuffer, cmd_soft_P, got) == 0) {
      if (arg) {
#ifdef SOFTSER
        uint32_t val = atol(arg);
        if (val > 0) {
          if (val > 38400) val = 38400;      //maximum speed softser can handle
          writeEP16(SOFTSER_BAUD, val / 10);
          changeSoftRate(val);
        }
#endif
      } else {
        print(F("Soft: "));
#ifdef SOFTSER
        println((uint32_t)readEP16(SOFTSER_BAUD) * 10);
#else
        println(0);
#endif
      }
      return;
    }

#ifdef DUMPVARS
    //(vars) Variables info.
    if (strncmp_P(cmdBuffer, cmd_vars_P, got) == 0) {
       dumpVars();
       return;
    }
#endif

#ifndef SOFTSER
    //(quit) Return to Mavlink mode
    if (strncmp_P(cmdBuffer, cmd_quit_P, got) == 0) {
      println(F("Resuming Mavlink mode.."));
      cli_active = 0;
      return;
    }
#endif

    if (strncmp_P(cmdBuffer, cmd_help_P, got) == 0) {
      println( F("List of commands:\r\n" 
      "version   \tMavPixel firmware version\r\n" 
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
#ifdef LAMPTEST
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
#ifndef SOFTSER
      "quit      \tExit CLI mode"
#endif
      ));
      return;
    }

    //Command unknown
    println( F("Unknown command"));

}
