
const char PROGMEM cmd_version_P[] = "version";
const char PROGMEM cmd_quit_P[] = "quit";
const char PROGMEM cmd_led_P[] = "led";
const char PROGMEM cmd_baud_P[] = "baud";
const char PROGMEM cmd_free_P[] = "free";

void enterCommandMode() {
  Serial.print(F("\r\nMavPixel " VER " ready.\r\n#"));
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

    //(led) Configure leds
    if (strncmp_P(cmdBuffer, cmd_led_P, got) == 0) {
      if (arg) {
        Serial.println(arg);
      } else {
        for (int i = 0; i < 32; i++) {
          Serial.print(F("led "));
          Serial.print(i);
          Serial.print(F(" "));
          printLedConfig(i);
          Serial.println();
        }
      }
      return;
    }

    //(baud) Configure Mavlink baud rate
    if (strncmp_P(cmdBuffer, cmd_baud_P, got) == 0) {
      if (arg) {
        Serial.println(arg);
        int val = atoi(arg);
        if (val > 0) {
          Serial.print(F("Setting baud."));
          writeEP16(MAVLINK_BAUD, val);
          changeBaudRate(val);
        }
      } else {
        uint16_t current = readEP16(MAVLINK_BAUD);
        Serial.print(F("Baud: "));
        Serial.println(current);
      }
      return;
    }

    //(v) Version info.
    if (strncmp_P(cmdBuffer, cmd_free_P, got) == 0) {
          Serial.print(F("Freemem: "));
          Serial.println(freeMem());
       return;
    }

    //(quit) Return to Mavlink mode
    if (strncmp_P(cmdBuffer, cmd_quit_P, got) == 0) {
      Serial.println(F("Resuming Mavlink mode."));
      cli_active = 0;
      return;
    }

    //Command unknown
    Serial.println( F("List of commands:\r\n" \
      "version   \tMavPixel firmware version.\r\n" \
      "led       \tConfigure LEDs.\r\n" \
      "baud      \tSet serial baud rate.\r\n" \
      "free      \tFree RAM.\r\n" \
      "help      \tThis list.\r\n"
      "quit      \tExit CLI mode."));
}
