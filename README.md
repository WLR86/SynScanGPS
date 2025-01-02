# SynScanGPS

Emulate a SynScan GPS using an Arduino and a cheap GPS

# Hardware
- Arduino Mini
- 12VDC -> 5VDC converter
- RS232/TTL adapter
- RJ cable to the SynScan multi-purpose port (12v & RS232)
- GPS module (with integrated TTL serial communication port - eg VK2828U8G5LF From V.KEL)

![Protocol](img/protocol.jpg) ![Build](img/arduino_gps.jpg)

Thanks to keymlinux for the latitude/longitude encoding

# Prerequisites

In order to make this work, the GPS module needs to be configured as follows:
4800 Baud
Output : GGA, RMC GSA and GSV messages must be activated
(AdaFruit GPS Library: https://github.com/adafruit/Adafruit_GPS is used and required - doc says only RMC and GGA are parsed)
