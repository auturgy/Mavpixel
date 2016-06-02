//Simple command interpreter

const char PROGMEM cmd_version_P[] = "version";
const char PROGMEM cmd_quit_P[] = "quit";
const char PROGMEM cmd_led_P[] = "led";
const char PROGMEM cmd_color_P[] = "color";
const char PROGMEM cmd_baud_P[] = "baud";
const char PROGMEM cmd_free_P[] = "free";
const char PROGMEM cmd_vars_P[] = "vars";
const char PROGMEM cmd_mode_P[] = "mode_color";
const char PROGMEM cmd_lbv_P[] = "lowcell";
const char PROGMEM cmd_lbp_P[] = "lowpct";
const char PROGMEM cmd_freset_P[] = "factory";



void enterCommandMode() {
  Serial.print(F("\r\nMavPixel " VER " ready.\r\n#"));
}

void printLed(uint8_t i) {
  Serial.print(F("led "));
  Serial.print(i);
  Serial.print(F(" "));
  printLedConfig(i);
  Serial.println();
}

void printColor(uint8_t i) {
  Serial.print(F("color "));
  Serial.print(i);
  Serial.print(F(" "));
  printColorConfig(i);
  Serial.println();
}

void printMode(uint8_t i) {
  Serial.print(F("mode_color "));
  Serial.print(i);
  Serial.print(F(" "));
  printModeConfig(i);
  Serial.println();
}

void doCommand() {
    char *cp, *arg;
    int got;
  
    //Chop off EOL.
    if ((cp = strchr(cmdBuffer, '\r')) != 0)
        *cp = 0;
    if ((cp = strchr(cmdBuffer, '\n')) != 0)
        *cp = 0;

    //Ignore blank lines.
    got = strlen(cmdBuffer);
    if (got == 0) return;
    
    //Get first word
    arg = strstr(cmdBuffer, " ");
    if (arg) {
      got = arg - cmdBuffer; //length of first word
      arg += 1; //ptr to argument
    }
  
    //(v) Version info.
    if (strncmp_P(cmdBuffer, cmd_version_P, got) == 0) {
       Serial.println(F("Version : " VER));
       return;
    }

#ifdef membug
    //(f) Free RAM.
    if (strncmp_P(cmdBuffer, cmd_free_P, got) == 0) {
          Serial.print(F("Free RAM: "));
          Serial.println(freeMem());
       return;
    }
#endif

#ifdef LED_STRIP
    //(led) Configure leds
    if (strncmp_P(cmdBuffer, cmd_led_P, got) == 0) {
      if (arg) {
        //Get second word
        cp = strstr(arg, " ");
        if (cp) *cp = 0;
        int i = atoi(arg);
        if (i < 0 || i > 31) Serial.println(F("Range error."));
        else if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          if (!parseLedStripConfig(i, cp)) Serial.println(F("Parse error."));
          else writeStruct(LED_CONFIGS, (uint8_t*)ledConfigs, sizeof(ledConfigs));  
        } else printLed(i);
      } else for (int i = 0; i < 32; i++) printLed(i);
      return;
    }

    //(color) Configure colors
    if (strncmp_P(cmdBuffer, cmd_color_P, got) == 0) {
      if (arg) {
        //Get second word
        cp = strstr(arg, " ");
        if (cp) *cp = 0;
        int i = atoi(arg);
        if (i < 0 || i > 15) Serial.println(F("Range error."));
        else if (cp) {
          got = arg - cmdBuffer; //length of first word
          cp += 1; //ptr to argument
          if (!parseColor(i, cp)) Serial.println(F("Parse error."));
          else writeStruct(COLOR_CONFIGS, (uint8_t*)colors, sizeof(colors));  
        } else printColor(i);
      } else for (int i = 0; i < 16; i++) printColor(i);
      return;
    }

    //(mode_color) Configure modes
    if (strncmp_P(cmdBuffer, cmd_mode_P, got) == 0) {
      if (arg) {
        //Get second word
        cp = strstr(arg, ",");
        if (!cp) {
          int i = atoi(arg);
          if (i < 0 || i > 20) Serial.println(F("Range error."));
          printMode(i);
        } else if (!parseMode(arg)) Serial.println(F("Parse error."));
      } else for (int i = 0; i <= 20; i++) printMode(i);
      return;
    }

    //(lowcell) Low battery cell voltage
    if (strncmp_P(cmdBuffer, cmd_lbv_P, got) == 0) {
      if (arg) {
        float val = (float)(atof(arg));
        lowBattVolt = val;
        writeEP16(LOWBATT_VOLT, val * 1000);
      } else {
        Serial.print(F("Low battery cell voltage: "));
        Serial.print(lowBattVolt);
        Serial.println(F("v"));
      }
      return;
    }

    //(lowpct) Low battery percentage
    if (strncmp_P(cmdBuffer, cmd_lbp_P, got) == 0) {
      if (arg) {
        uint8_t val = atoi(arg);
        lowBattPct = val;
        writeEEPROM(LOWBATT_PCT, val);
      } else {
        Serial.print(F("Low battery percentage: "));
        Serial.print(lowBattPct);
        Serial.println(F("%"));
      }
      return;
    }

    //(factory) Factory Reset.
    if (got == 7 && strncmp_P(cmdBuffer, cmd_freset_P, got) == 0) {
       writeFactorySettings();
       Serial.println(F("Please reset Mavpixel."));
    }

#endif

    //(baud) Configure Mavlink baud rate
    if (strncmp_P(cmdBuffer, cmd_baud_P, got) == 0) {
      if (arg) {
        uint32_t val = atoi(arg);
        if (val > 0) {
          Serial.print(F("Setting baud."));
          writeEP16(MAVLINK_BAUD, val / 10);
          changeBaudRate(val);
        }
      } else {
        uint32_t current = readEP16(MAVLINK_BAUD) * 10;
        Serial.print(F("Baud: "));
        Serial.println(current);
      }
      return;
    }

#ifdef JD_IO
    //(vars) Variables info.
    if (strncmp_P(cmdBuffer, cmd_vars_P, got) == 0) {
       dumpVars();
       return;
    }
#endif

    //(quit) Return to Mavlink mode
    if (strncmp_P(cmdBuffer, cmd_quit_P, got) == 0) {
      Serial.println(F("Resuming Mavlink mode."));
      cli_active = 0;
      return;
    }

    //Command unknown
    Serial.println( F("List of commands:\r\n" 
      "version   \tMavPixel firmware version.\r\n" 
#ifdef LED_STRIP
      "led       \tConfigure LEDs.\r\n" 
      "color     \tConfigure colours.\r\n" 
      "mode_color\tConfigure colors for modes\r\n"
      "lowcell   \tLow battery cell voltage\r\n"
      "lowpct    \tLow battery percentage\r\n"
      "factory   \tFactory reset\r\n"
#endif
      "baud      \tSet serial baud rate.\r\n" 
#ifdef JD_IO
      "vars      \tDump variables.\r\n" 
#endif
#ifdef membug
      "free      \tFree RAM.\r\n" 
#endif
      "help      \tThis list.\r\n"
      "quit      \tExit CLI mode."));
}
