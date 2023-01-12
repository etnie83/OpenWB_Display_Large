This is a fork of MartinRinas openWB_OLEDDisplay
but with the change to display the housebattery and the direction of the power

# openWB_OLEDDisplay
OpenWB status display using ESP8266 and x.xx" OLED Display.
Displays current EVU, PV and combined power of charging port plus SoC of charge port 1

Sketch assumes SPI Display, wiring for Wemos D1 & compatible:

            Wemos   Display
              |        |
      GND:   GND       1
      VCC:   5V        2
      SCK:   Dx        3
      SDA:   Dx        4
      RES:   D6        5
      RS:    D1        6
      CS:    D8        7
      LEDA:  3V3       8

# Configuration
You need to enter SSID, PW and IP of openWB in .ino source file
