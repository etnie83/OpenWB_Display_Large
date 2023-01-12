This is a fork of MartinRinas openWB_OLEDDisplay
but with the change to display the housebattery and the direction of the power on a bigger display

# openWB_OLEDDisplay
OpenWB status display using ESP8266 and x.xx" OLED Display.
Displays current EVU, PV and combined power of charging port plus SoC of charge port 1

Sketch assumes Display, wiring for Wemos D1 & compatible:

            Wemos   Display
              |        |
      GND:   GND       1
      VCC:   5V        2
      SCK:   D5*       3
      SDA:   D7*       4
      RES:   D6        5
      RS:    D1        6
      CS:    D8        7
      LEDA:  3V3       8

            * or switched :-)

# Configuration
You need to enter SSID, PW and IP of openWB in .ino source file

<img src="/images/display.jpg" style="display: inline-block; margin: 0 auto; height: 400px">
