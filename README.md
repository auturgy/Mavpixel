Mavpixel
========

*Under Construction..*

Mavpixel brings Cleanflight-style LED strip functionality to the APM project.

Mavpixel is a LED strip controller designed as a companion for APM, Pixhawk and other Mavlink-compatible flight controllers. Based on simple, inexpensive hardware, Mavpixel is a well-featured complete LED lighting solution for larger UAVs.

Mavpixel uses no special hardware. An Arduino Pro Mini with programmer, some WS2812 LED strips and a handfull of wire is all the hardware required.

Mavpixel can control up to 32 RGB LEDs, with functions including: 

* Flight modes
* Arming status 
* Warnings like low battery and failsafe
* Indicators
* Throttle position
* GPS status with number of satellites
* Orientation
* Fixed colours
* Animation effects
* Thrust ring

Mavpixel takes advantage of the LED strip functionality found in the Cleanflight flight controller firmware. This was reconfigured to fit into a small Arduino and adapted to be controlled over a Mavlink connection.

The Mavpixel has a very flexible configuration system. It is a fully supported Mavlink peripheral and can be set up over any available communication channel, even through the flight controller. Everything except firmware upgrades can be done through Mavlink, allowing LED strip configuration over the flight controller's USB or telemetry and through Mission Planner, Mavproxy or other ground stations with support for UDP packet forwarding.

As Mavpixel can also use a command line interface for configuration it can be set up without anything more than a serial terminal connected to the programming port (57600 baud) or optionally connected to the Software Serial pins (2400 baud) as described below.

For ease of use the simple configuration application MavpixelGUI is also provided with layout and controls familiar to users of Cleanflight. This acts as a minimal ground station and can connect to any available serial or network port or through a ground station in either Mavlink or CLI mode as appropriate. It also includes a firmware flasher for preparing and keeping Mavpixel boards up to date.

Some ground stations are able to configure Mavpixel's basic parameters from within the ground station itself but due to what is currently somewhat poor support in many ground stations for Mavlink peripherals this feature has been left disabled by default. To try it out, enable 'heartbeats' in the Mavpixel's settings.

**Basic system diagram**

![Mavpixel wiring diagram](https://github.com/prickle/Mavpixel/raw/master/images/Mavpixel_wiring.png)

*Diagram Notes: Mavpixel can use Telem 1, Telem 2, Serial 4 or 5. Only use servo rail power if enough current is available.*

**Quick start guide**

**Step 1)** Obtain hardware.

This application currently supports Mavpixel on Arduino Pro Mini. 
Obtain an Arduino Pro Mini 5v 16Mhz board along with an FTDI programmer.

**Step 2)** Download and start MavpixelGUI.

MavpixelGUI is used to prepare and configure Arduino boards to become Mavpixels.
Download from [here](http://github.com/prickle/MavpixelGUI/releases)

The latest source code for MavpixelGUI can be found at http://github.com/prickle/MavpixelGUI

**Step 3)** Flash the Mavpixel firmware.

Connect the Arduino Pro Mini using an FTDI programmer to a USB port. Use the Firmware Flasher in MavpixelGUI to flash the Arduino with the latest Mavpixel firmware. The Arduino is now a Mavpixel, ready to connect on the same port at 57600 baud for offline setup if desired.

The source code and firmware files for Mavpixel can be found at http://github.com/prickle/Mavpixel

**Step 4)** Connect the LEDs.

Mavpixel currently supports WS2812 LED strips in lengths of no more than eight.
Four LED strip outputs are available which can drive up to eight LEDs each, giving a maximum of 32 LEDs. This configuration was used due to Arduino hardware limitations that prevented driving one long strip.

Power for the strips should be taken from a dedicated 5 volt UBEC or from the flight controller's servo rail if enough current is available. Do not try to power the strips from the Mavpixel as they may try to draw too much power from the USB connection and fry the flight controller. 

The LED strips require up to two amps at 5 volts for all 32 LEDs at full brightness white, usually much less but the overhead needs to be there or problems will ensue.

Run the data line from each strip to the Mavpixel pins A1(leds 0-7), A2(leds 8-15), A3(leds 16-23), and A4(leds 24-31).

**Step 5)** Connect to the flight controller

Connect a spare telemetry port on the flight controller to the Mavpixel's serial port through it's programming connector using four wires +5v, gnd, TxD->RxD and RxD->TxD. Power for the Mavpixel is taken from the flight controller so it is powered and configurable when the vehicle is connected to USB.

**Step 6)** Configure the Mavpixel

Use MavpixelGUI to connect directly to the flight controller's serial or network port for quick online setup. Note that changes made in the GUI are activated on the Mavpixel only after the Send button at the bottom of the window is pressed.

**Troubleshooting**

*No Reactions - communication problems

First check to see if Mavpixel can hear Mavlink. This is done by examining the Mavpixel's onboard status LED. A rapid flashing (5 Hz) means no Mavlink is being received. A slower flash (1 Hz) is what you want to see. Note that the flight controller takes a few seconds after booting before it begins emitting Mavlink messages so be patient.

If Mavpixel indicates there is no Mavlink, confirm your flight controller's `SERIALx` settings on the appropriate port (i.e. `SERIAL2` for Telem 2 port). These should be set to `SERIAL_BAUD = 57` and `SERIAL_PROTOCOL = 1`. Still no luck? Double check all connections between Mavpixel and the flight controller.

If Mavpixel indicates Mavlink is being received but the LEDs are not reacting, check the flight controller's `SRx` setting for the port Mavpixel is connected to (i.e. `SR2` for Telem 2). Mavpixel sets these Stream Rate parameters itself, so if Mavlink communication is working in both directions one should see `SRx_EXT_STAT = 2`, `SRx_EXTRA_2 = 5`, and `SRx_RC_CHAN = 5`.

All other `SRx` settings should be 0 as Mavpixel does not use them. If they are not, it could be settings left over from a MinimOSD or the like. Try setting all `SRx` values to 0, wait a moment, then refresh params. Mavpixel should have set it's three `SRx` parameters back to 2, 5 and 5.

If `SRx` parameters do not change or stay at 0, Mavpixel cannot talk back to the flight controller. Check the telemetry port connections again.

No luck? If necessary Mavpixel can operate on a half-duplex connection. 

*Flickering/odd colours - power supply probems

The LED strips require up to two amps at 5 volts for all 32 LEDs at full brightness white. If the current draw cannot be satisfied the voltage will droop and the LEDs will start to misbehave.

Try reducing the brightness in Mavpixel's settings. This also reduces the current draw. If the LEDs start acting normally then the LED's power supply cannot provide enough current.

If reducing brightness does not help there could be issues from radiated noise, voltage drop, ground loops or the like. Get in touch if you are experience something like this.

**Half duplex connection**

The disadvantage of a half-duplex connection is being unable to configure Mavpixel through Mavlink. The advantage is being able to piggyback off another telemetry connection, for instance a SiK radio, and not require a dedicated serial port.

With a half-duplex connection the TxD line from the Mavpixel to the flight controller is left out. `SRx` settings on the flight controller will then need to be set manually to `SRx_EXT_STAT = 2`, `SRx_EXTRA_2 = 5`, and `SRx_RC_CHAN = 5` or Mavpixel will not react if a ground station is not connected.

**Software Serial configuration port**

When Mavlink configuration is unavailable or CLI access is desired while Mavlink is connected, Mavpixel configuration can be done live through Mavpixel's secondary Software Serial configuration port. 

![Software Serial wiring diagram](https://github.com/prickle/Mavpixel/raw/master/images/Mavpixel_softserial.png)

Connect an FTDI or equivalent USB-to-serial converter's TxD to Mavpixel pin 8, RxD to pin 9 and GND to a spare ground pin. The Arduino Pro Mini's FTDI programmer is perfectly suitable for this. Open the port with a terminal or MavpixelGUI at 2400 baud for full configuration access.

Although it is possible to increase the speed of the Software Serial configuration port up to 38400 baud it is recommended to keep it at 2400 baud to ensure reliability as it has a tendency to drop bits at higher rates. 

This is due to hardware limitations. Software Serial is interrupt driven and needs to catch every bit in the serial data stream. Because of the tight timing requirements of the LED strips they can disable interrupts for longer than a fast serial bit time. This causes occasional bits to be lost from the serial data stream and sometimes corrupts the message. Keeping the Software Serial baud rate low ensures serial data bit transitions are not missed.

**Developer notes**

Mavpixel is a regular Arduino sketch that can be opened, compiled and uploaded with the Arduino IDE version 1.0.6. Later versions will not work with the FastSerial libraries. Broad compilation options can be found near the top of Mavpixel.ino. Program constants holding Mavpixel's identity and characteristics are in IOBoard.h. Factory defaults are in IO_EEPROM.ino. Mavlink heartbeat, rate requests, reader and parameter getter/setters are all in MAVLink.ino.

The Arduino IDE's Serial Monitor works well with Mavpixel. Set line endings to 'Both NL & CR' and (by default) connect at 57600 baud.

**Acknowledgements**

Mavpixel would like to thank it's creators, authors and contributors. In particular:

[jD-IOBoard](http://github.com/jdrones/jD-IOBoard) - Jani Hirvinen along with co-authors Sandro Beningo and Mike Smith.

[Cleanflight](http://github.com/cleanflight/cleanflight) (ledstrip.c) - Dominic Clifton, Petr Ledvina, Gaël James and many others.

[AltSoftSerial](http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html) - Paul Stoffregen, paul@pjrc.com.

[Adafruit](http://github.com/adafruit/Adafruit_NeoPixel) - Adafruit_NeoPixel libraries.

[Mavlink](http://qgroundcontrol.org/mavlink/start) - Lorenz Meier, Andrew Tridgell and many others.

And of course, 3D Robotics and the DIYDrones community.

Apologies to any I may have forgotten or missed. If you belong here let me know.

This software is released under the open source GPL license. For more details, see LICENSE in the application directory.
