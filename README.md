Mavpixel
========
Mavpixel brings Cleanflight-style LED strip functionality to the APM project.

A recent version of the LED strip driver source code was taken from Cleanflight and adapted to run on an Arduino.

The Mavlink interpreter demo from the jD-IOBoard LED controller was combined with the Cleanflight LED driver to allow the strip to be controlled over a Mavlink serial connection.

This project is pre-alpha and is currently still in development.




What is jD-IOBoard??

jD-IOBoard is small standalone Arduino board that has 6 high power outputs and several TTL level ouputs. jD-IOBoard 
is designed for simple I/O, LED etc driving needs. You can connect up to 42v/500mAh per highpower output pins.

Technical details:
- Arduino Mini Pro compatible
- 1 x UART serial
- 2 x Software serial pins exposed
- 1 x I2C pins exposed
- 5 x Analog pins exposed
- 6 x high power pins (42v/500mAh per pin)
- 6 x TTL level pins


For more information, documentation and installation, please take a look at http://www.jDrones.com/documents


Boards can be found from http://store.jdrones.com/jD_IOBoard_p/jdioboard11.htm


