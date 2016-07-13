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

**Quick start guide**

**Step 1)** Obtain hardware
This application currently supports Mavpixel on Arduino Pro Mini.

Obtain an Arduino Pro Mini 5v 16Mhz board along with an FTDI programmer.

**Step 2)** Download and start MavpixelGUI.

MavpixelGUI is used to prepare and configure Arduino boards to become Mavpixels.
Download from [here](http://github.com/prickle/MavpixelGUI/releases)

The source code and firmware files for MavpixelGUI can be found at http://github.com/prickle/MavpixelGUI

**Step 3)** Flash the Mavpixel firmware.

Connect the Arduino Pro Mini using an FTDI programmer to a USB port. Use the Firmware Flasher in MavpixelGUI to flash the Arduino with the latest Mavpixel firmware. The Arduino is now a Mavpixel, ready to connect on the same port at 57600 baud for offline setup if desired.

The source code and firmware files for Mavpixel can be found at http://github.com/prickle/Mavpixel

**Step 4)** Connect the LEDs
Mavpixel currently supports WS2812 LED strips in lengths of no more than eight.

Four LED strip outputs are available which can drive up to eight LEDs each, giving a maximum of 32 LEDs. This configuration was used due to Arduino hardware limitations that prevented driving one long strip.

Power for the strips should be taken from a dedicated 5 volt UBEC or from the flight controller's servo rail if enough current is available. Do not try to power the strips from the Mavpixel as they may try to draw too much power from the USB connection and fry the flight controller.

**Step 5)** Connect to the flight controller

Connect a spare telemetry port on the flight controller to the Mavpixel's serial port throug it's programming connector using four wires +5v, gnd, txd->rxd and rxd->txd. Power for the Mavpixel is taken from the flight controller so it is powered and configurable when the vehicle is connected to USB.

**Step 6)** Configure the Mavpixel

Use MavpixelGUI to connect directly to the flight controller's serial or network port for quick online setup. Note that changes made in the GUI are activated on the Mavpixel only after the Send button at the bottom of the window is pressed.

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
