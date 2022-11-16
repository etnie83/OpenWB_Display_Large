#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PubSubClient.h>
#include <Wire.h>
//#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#define TFT_CS        D8
#define TFT_RST       D6 
#define TFT_DC        D1

Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Global constants for WiFi connections
// ***********************************
// Need to replace with WiFiManager
// ***********************************

// Network setup
const char* ssid = "SSID";              // your network SSID (name)
const char* pass = "PASSWORD";        // your network password
const char* hostname = "openWB-DisplayLarge";      

// MQTT Setup
IPAddress MQTT_Broker(192,168,0,105); // openWB IP address
const int MQTT_Broker_Port = 1883;
bool initScreen = true;
// MQTT topics and variables for retrieved values
const char* MQTT_EVU_W = "openWB/evu/W";    // current power at EVU
int EVU_W[] = {0, 0};
int EVU_dir[] = {1, 1};
int HB_dir = 1;
unsigned long previousMillisTimer = 0; 
const long interval = 500; 
bool NewData = false;
bool ErrorWasActive = false;
int batteryRedPercent = 10;
int batteryYellowPercent = 35;

const char* MQTT_PV_W = "openWB/pv/W";      // current PV power
int PV_W[] = {0, 0};

const char* MQTT_LP_all_W= "openWB/global/WAllChargePoints";  // current power draw for all charge points
int LP_all_W[] = {0, 0};

const char* MQTT_LP1_SOC= "openWB/lp/1/%Soc";  // current power draw for all charge points
int LP1_SOC[] = {0, 0};

const char* MQTT_LP1_PlugStat = "openWB/lp/1/boolPlugStat"; // is the car plugged in?
bool LP1_PlugStat[] = {false, false};

const char* MQTT_LP1_IsCharging = "openWB/lp/1/boolChargeStat"; // charging active?
bool LP1_IsCharging = false;

const char* MQTT_HB_W = "openWB/housebattery/W"; // HouseBattery Charge/Discharge
int HB_W[] = {0, 0};

const char* MQTT_HB_SOC = "openWB/housebattery/%Soc"; // HouseBattery Charge/Discharge
int HB_SOC[] = {0, 0};

const char* MQTT_HOUSE_W = "openWB/global/WHouseConsumption"; // House Load
int HOUSE_W[] = {0, 0};


// Display Setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 160 // OLED display height, in pixels

#define shift_k_value  3
#define shift_dot  1

const uint8_t blitz[10] = { 0x3c, 0x78, 0x70, 0xe0,
                            0xfc, 0x38, 0x30, 0x60,
                            0xc0, 0x80};
const uint8_t arrow_right[10] = { 0x00, 0x08, 0x0c, 0x0e,
                                  0xff, 0xff, 0x0e, 0x0c,
                                  0x08, 0x00 };
const uint8_t arrow_left[10] = { 0x00, 0x10, 0x30, 0x70,
                                 0xff, 0xff, 0x70, 0x30,
                                 0x10, 0x00 };
const uint8_t battery[10] = { 0x00, 0x00, 0x66, 0xff,
                              0xff, 0xdf, 0x89, 0xdf,
                              0xff,0xff};
const uint8_t haus[15] = { 0x0c, 0x01, 0xe0, 0x3f, 
                           0x07, 0xf8, 0xff, 0xcc, 
                           0xcc, 0xcc, 0xcf, 0xcc,
                           0xfc, 0xcf, 0xcc};
const uint8_t haus2[20] = { 0x0c, 0x00, 0x1e, 0x00, 
                            0x3f, 0x00, 0x7f, 0x80,
                            0xff, 0xc0, 0xcc, 0xc0,
                            0xcc, 0xc0, 0xfc, 0xc0,
                            0xfc, 0xc0, 0xfc, 0xc0 };
const uint8_t unplugged[30] = { 0x00, 0x00, 0x00,
                                0xf0, 0x00, 0x00,
                                0xb0, 0x00, 0x00,
                                0xb0, 0x60, 0x00,
                                0x90, 0x40, 0x00,
                                0xde, 0x40, 0x00,
                                0xd2, 0x40, 0x00,
                                0xf2, 0x40, 0x00,
                                0xf3, 0xc0, 0x00,
                                0xf0, 0x00, 0x00 };
const uint8_t plugged[30] = { 0x00, 0x07, 0xf0,
                              0xf0, 0x04, 0x10,
                              0xb0, 0x08, 0x08,
                              0xb0, 0x08, 0x08,
                              0x90, 0x38, 0x0e,
                              0xde, 0x0f, 0xf8,
                              0xd2, 0x79, 0xc8,
                              0xf2, 0x4f, 0xf8,
                              0xf3, 0xcf, 0xf8,
                              0xf0, 0x0c, 0x18 };

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     0 // Reset pin # (or -1 if sharing Arduino reset pin)

unsigned long currentMillis;
unsigned long previousMillis = 0;         // last time data was fetched
unsigned long lastMQTTDataReceived = 0;
int MaxDataAge = 30*1000; // max wait time for new data from MQTT subscription

WiFiClient espClient;
PubSubClient MQTTClient(espClient);
long lastReconnectAttempt = 0; // WiFi Reconnection timer

// Config flags do enable features
const bool isDebug = 1;                        // Send debug messages to serial port?

// ESP8266 Webserver and update server
ESP8266WebServer server(80);              // HTTP server port
ESP8266HTTPUpdateServer httpUpdater;      // HTTP update server, allows OTA flash by navigating to http://<ESP8266IP>/update

void WriteLog(String msg,bool NewLine=1)  // helper function for logging, only write to serial if isDebug is true
{
  if(NewLine)
  {
    if(isDebug){Serial.println(msg);}
  }
  else
  {
    if(isDebug){Serial.print(msg);}
  } 
}

boolean MQTTReconnect() 
{
  if (MQTTClient.connect(hostname)) 
  {
    WriteLog("MQTT Reconnected");
    boolean r = MQTTClient.subscribe(MQTT_EVU_W);
    if (r)
    {
        WriteLog("MQTT subscription suceeded");
    }
    else
    {
        WriteLog("MQTT subscription failed");
    }
    
    r = MQTTClient.subscribe(MQTT_LP_all_W);
    r = MQTTClient.subscribe(MQTT_PV_W);
    r = MQTTClient.subscribe(MQTT_LP1_SOC);
    r = MQTTClient.subscribe(MQTT_LP1_IsCharging);
    r = MQTTClient.subscribe(MQTT_LP1_PlugStat);
    r = MQTTClient.subscribe(MQTT_HB_W);
    r = MQTTClient.subscribe(MQTT_HB_SOC);
    r = MQTTClient.subscribe(MQTT_HOUSE_W);
  }
  return MQTTClient.connected();
}

void HandleRoot()                                                 // Handle Webserver request on root
{
  String res = "HTTP Server up and running.";
  WebserverResponse(res);
}

void HandleMQTTStatus()
{
  String res = String(MQTTClient.state());
  WebserverResponse(res);
}

void WebserverResponse(String str)
{ 
    str.trim();
    WriteLog("Sending WebServer response, requested URI: " + server.uri());
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "text/plain",String(str));
    WriteLog("Sending HTTP response: " + str);
}

void MQTTCallback(char* topic, byte* payload, unsigned int length) 
{
  lastMQTTDataReceived = millis();
  WriteLog("Message arrived: [" ,0);
  WriteLog(topic ,0);
  WriteLog("]" ,0);
  String msg;
  for (int i=0;i<length;i++) { // extract payload
    msg = msg + (char)payload[i];
  }
  WriteLog(msg);
  
  // store values in variables
  // todo use MQTT_ constants instead of hard coded values to compare
  if (strcmp(topic,"openWB/evu/W")==0){ EVU_W[0] = (msg.toInt()); EVU_dir[0] = 1;
                                        if (EVU_W[0] < 0)
                                        {
                                           EVU_W[0] = EVU_W[0]*(-1);
                                           EVU_dir[0] = -1;
                                        }
                                      }
  if (strcmp(topic,"openWB/pv/W")==0){PV_W[0] = (msg.toInt()*-1);}
  if (strcmp(topic,"openWB/global/WAllChargePoints")==0){LP_all_W[0] = msg.toInt();}
  if (strcmp(topic,"openWB/lp/1/%Soc")==0){LP1_SOC[0] = msg.toInt();}
  if (strcmp(topic,"openWB/lp/1/boolChargeStat")==0){LP1_IsCharging = msg.toInt();}
  if (strcmp(topic,"openWB/lp/1/boolPlugStat")==0){LP1_PlugStat[0] = msg.toInt();}
  if (strcmp(topic,"openWB/housebattery/W")==0){ HB_W[0] = (msg.toInt()); HB_dir = 1;
                                        if (HB_W[0] < 0)
                                        {
                                           HB_W[0] = HB_W[0]*(-1);
                                           HB_dir = -1;
                                        }}
  if (strcmp(topic,"openWB/housebattery/%Soc")==0){HB_SOC[0] = msg.toInt();}
  if (strcmp(topic,"openWB/global/WHouseConsumption")==0){HOUSE_W[0] = msg.toInt();}
  
  // processed incoming message, lets update the display
  //UpdateDisplay();
  NewData = true;
}

void WriteDisplayNewText(String msg)
{
  display.setCursor(0,0);
  display.fillScreen(ST77XX_BLACK);
  display.setTextSize(2);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(0,0);
  WriteDisplayText(msg);
}

void WriteDisplayText(String msg)
{
  display.println(msg);
}

void WriteWattValue(int Watt, int x, int y, uint16_t color, int textsize = 2)
{
  int charsize;
  switch (textsize) {
    case 1:
      charsize = textsize*6 + 1;
      break;
    case 2:
      charsize = textsize*6;
      break;
    default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
  }
  // check if Watt Value is smaller than 1000 (=1kW)
  if (Watt < 1000)
  {
  // value is smaller than 1kW, 
  // need to write value right-aligned
    if (Watt < 10)
    {
      display.setCursor(x-1*charsize, y);
    }
    else if (Watt < 100)
    {
      display.setCursor(x-2*charsize, y);
    }
    else
    {
      display.setCursor(x-3*charsize, y);
    }
    display.setTextColor(color);
    display.println(String(Watt));
  }
  else
  {
    if (Watt < 10000)
    {
      int D_Watt_kW=Watt/1000;
      int D_Watt_W=Watt%1000;
      if (D_Watt_W <10)
      {
        display.setCursor(x-3*charsize, y);
        display.setTextColor(color);
        display.print("00"+String(D_Watt_W));
        display.setCursor(x-4*charsize+shift_dot, y);
        display.print(".");
        display.setCursor(x-5*charsize+shift_k_value+shift_dot, y);
        display.print(String(D_Watt_kW));
      }
      else if (D_Watt_W < 100)
      {
        display.setCursor(x-3*charsize, y);
        display.setTextColor(color);
        display.print("0"+String(D_Watt_W));
        display.setCursor(x-4*charsize+shift_dot, y);
        display.print(".");
        display.setCursor(x-5*charsize+shift_k_value+shift_dot, y);
        display.print(String(D_Watt_kW));
      }
      else
      {
        display.setCursor(x-3*charsize, y);
        display.setTextColor(color);
        display.print(String(D_Watt_W));
        display.setCursor(x-4*charsize+shift_dot, y);
        display.print(".");
        display.setCursor(x-5*charsize+shift_k_value+shift_dot, y);
        display.print(String(D_Watt_kW));
      }
    }
    else
    {
      int D_Watt_kW=Watt/1000;
      int D_Watt_W=(Watt%1000)/10;
      if (D_Watt_W <100)
      {
        display.setCursor(x-2*charsize, y);
        display.setTextColor(color);
        display.print("0"+String(D_Watt_W));
        display.setCursor(x-3*charsize+shift_dot, y);
        display.print(".");
        display.setCursor(x-5*charsize+shift_k_value+shift_dot, y);
        display.print(String(D_Watt_kW));
      }
      else
      {
        display.setCursor(x-2*charsize, y);
        display.setTextColor(color);
        display.print(String(D_Watt_W));
        display.setCursor(x-3*charsize+shift_dot, y);
        display.print(".");
        display.setCursor(x-5*charsize+shift_k_value+shift_dot, y);
        display.print(String(D_Watt_kW));
      }
    }
  }
}

void drawBitmap(uint16_t x, uint16_t y, uint8_t bitmap[], uint16_t w, uint16_t h)
{
  for (int i=0; i<w; i++)
  {
    for (int j=0; j<h; i++)
    {
      uint16_t bitindex = i+j*h;
      uint16_t byteindex = bitindex/8;
      uint8_t bytebitindex = bitindex % 8;
      uint16_t color = (bitmap[byteindex] >> bytebitindex) & 0x01;
      //drawPixel(x+i, y+j, color);
    }
  }
}


void UpdateDisplay()
{
  // for a 128*64px display:
  // Text Size 1: single char 6*8px, 21 chars per row, 8 rows 
  // Text Size 2: single char 12*16px, 10 chars per row, 4 rows 
  // Text Size 3: single char 18*24x, 7 chars per row, 2.5 rows 
  // Text Size 4: single char 24*32x, 5 chars per row, 2 rows 
  // Text Size 8: single char 48*64x, 2 chars per row, 1 row 
  if (initScreen)
  {
  display.setCursor(0,0);
  display.fillScreen(ST77XX_BLACK);
  }

  display.setCursor(0,0); //set upper left corner of cursor to upper left corner of display

  display.setTextSize(1);

  if (HOUSE_W[0] != HOUSE_W[1] || initScreen )
  {
    WriteWattValue(HOUSE_W[1], SCREEN_WIDTH/3*2-shift_k_value-shift_dot, 50, ST77XX_BLACK, 1);
    WriteWattValue(HOUSE_W[0], SCREEN_WIDTH/3*2-shift_k_value-shift_dot, 50, ST77XX_WHITE, 1);
    HOUSE_W[1] = HOUSE_W[0];
  }

  // check if EVU power is smaller than 1 kW
  if (EVU_W[0] < 1000 && (EVU_W[0] != EVU_W[1] || initScreen ))
  {
    display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,0); // Text size 1 has width of 6
    display.setTextColor(ST77XX_BLACK);
    display.println("EVU (kW)");
    // display description and value in W
    display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,0); // Text size 1 has width of 6
    display.setTextColor(ST77XX_WHITE);
    display.println("EVU (W)");
  }
  else if (EVU_W[0] != EVU_W[1] || initScreen)
  {
    display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,0); // Text size 1 has width of 6
    display.setTextColor(ST77XX_BLACK);
    display.println("EVU (W)");
    // display description and value in kW
    display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,0); // Text size 1 has width of 6
    display.setTextColor(ST77XX_WHITE);
    display.println("EVU (kW)");
  }
  
  if (EVU_W[0] != EVU_W[1] || initScreen)
    {
      display.setTextSize(2);
      WriteWattValue(EVU_W[1], SCREEN_WIDTH/2-shift_k_value-shift_dot, 10, ST77XX_BLACK);
      if (EVU_dir >= 0)
      {
      WriteWattValue(EVU_W[0], SCREEN_WIDTH/2-shift_k_value-shift_dot, 10, ST77XX_RED);
      }
      else if (EVU_dir < 0)
      {
      WriteWattValue(EVU_W[0], SCREEN_WIDTH/2-shift_k_value-shift_dot, 10, ST77XX_GREEN);
      }
      EVU_W[1] = EVU_W[0];
    }

  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH/2+2*6,0); // Text size 1 has width of 6
  // check if PV power is smaller than 1 kW
  if (PV_W[0] < 1000 && (PV_W[0] != PV_W[1] || initScreen))
  {
    display.setTextColor(ST77XX_BLACK);
    display.println("PV (kW)");
    // display description and value in W
    display.setCursor(SCREEN_WIDTH/2+2*6,0); // Text size 1 has width of 6
    display.setTextColor(ST77XX_WHITE);
    display.println("PV (W)");
  }
  else if (PV_W[0] != PV_W[1] || initScreen)
  {
    display.setTextColor(ST77XX_BLACK);
    display.println("PV (W)");
    // display description and value in kW
    display.setCursor(SCREEN_WIDTH/2+2*6,0); // Text size 1 has width of 6
    display.setTextColor(ST77XX_WHITE);
    display.println("PV (kW)");
  }
  if(PV_W[0] != PV_W[1] || initScreen)
    {
      display.setTextSize(2);
      display.setTextColor(ST77XX_WHITE);
      WriteWattValue(PV_W[1], SCREEN_WIDTH, 10, ST77XX_BLACK);
      WriteWattValue(PV_W[0], SCREEN_WIDTH, 10, ST77XX_WHITE);
      PV_W[1] = PV_W[0];
    }

  // Show HouseBattery
  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,SCREEN_HEIGHT/3*1+15); // Text size 1 has width of 6
    if (HB_W[0] < 1000 && (HB_W[0] != HB_W[1] || initScreen))
    {
      display.setTextColor(ST77XX_BLACK);
      display.println("HB (kW)");
      // display description and value in W
      display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,SCREEN_HEIGHT/3*1+15); // Text size 1 has width of 6
      display.setTextColor(ST77XX_WHITE);
      display.println("HB (W)");
    }
    else if (HB_W[0] != HB_W[1] || initScreen)
    {
      display.setTextColor(ST77XX_BLACK);
      display.println("HB (W)");
      // display description and value in kW
      display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,SCREEN_HEIGHT/3*1+15); // Text size 1 has width of 6
      display.setTextColor(ST77XX_WHITE);
      display.println("HB (kW)");
    }
    if (HB_W[0] != HB_W[1] || initScreen)
      {
        display.setTextSize(2);
        WriteWattValue(HB_W[1], SCREEN_WIDTH/2-shift_k_value-shift_dot, SCREEN_HEIGHT/3*1+25, ST77XX_BLACK);
        if(HB_dir > 0) 
          {
            WriteWattValue(HB_W[0], SCREEN_WIDTH/2-shift_k_value-shift_dot, SCREEN_HEIGHT/3*1+25, ST77XX_GREEN);
          }
        else if(HB_dir < 0) 
          {
            WriteWattValue(HB_W[0], SCREEN_WIDTH/2-shift_k_value-shift_dot, SCREEN_HEIGHT/3*1+25, ST77XX_RED);
          }
        else if(HB_W[0] == 0)
          {
            WriteWattValue(HB_W[0], SCREEN_WIDTH/2-shift_k_value-shift_dot, SCREEN_HEIGHT/3*1+25, ST77XX_WHITE);
          }
        HB_W[1] = HB_W[0]; 
      }

    display.setTextSize(2);
    if (HB_SOC[0] != HB_SOC[1] || initScreen)
    {
      display.setTextSize(1);
      display.setTextColor(ST77XX_WHITE);
      display.setCursor(SCREEN_WIDTH/2+2*6,SCREEN_HEIGHT/3*1+15); // Text size 1 has width of 6
      display.println("SoC HB"); // Charge Status as Symbol also available, could be removed from this line
      display.setTextSize(2);
      if (HB_SOC[1] < 10)
      {
        display.setCursor(SCREEN_WIDTH-2*12,SCREEN_HEIGHT/3*1+25);
        display.setTextColor(ST77XX_BLACK);
        display.print(String(HB_SOC[1])+"%");
      }
      else if (HB_SOC[1] < 100)
      {
        display.setCursor(SCREEN_WIDTH-3*12,SCREEN_HEIGHT/3*1+25);
        display.setTextColor(ST77XX_BLACK);
        display.print(String(HB_SOC[1])+"%");
      }
      else
      {
        display.setCursor(SCREEN_WIDTH-4*12,SCREEN_HEIGHT/3*1+25);
        display.setTextColor(ST77XX_BLACK);
        display.print(String(HB_SOC[1])+"%");
      }
      if (HB_SOC[0] < 10)
      {
        display.setCursor(SCREEN_WIDTH-2*12,SCREEN_HEIGHT/3*1+25);
        display.setTextColor(ST77XX_WHITE);
        display.print(String(HB_SOC[0])+"%");
        drawBar(SCREEN_HEIGHT/2+15, HB_SOC[0]);
      }
      else if (HB_SOC[0] < 100)
      {
        display.setCursor(SCREEN_WIDTH-3*12,SCREEN_HEIGHT/3*1+25);
        display.setTextColor(ST77XX_WHITE);
        display.print(String(HB_SOC[0])+"%");
        drawBar(SCREEN_HEIGHT/2+15, HB_SOC[0]);
      }
      else
      {
        display.setCursor(SCREEN_WIDTH-4*12,SCREEN_HEIGHT/3*1+25);
        display.setTextColor(ST77XX_WHITE);
        display.print(String(HB_SOC[0])+"%");
        drawBar(SCREEN_HEIGHT/2+15, HB_SOC[0]);
      }
    HB_SOC[1] = HB_SOC[0];
    }


// Show LP
    display.setTextSize(1);
    display.setTextColor(ST77XX_WHITE);
    display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,SCREEN_HEIGHT/3*2+19); // Text size 1 has width of 6
    if (LP_all_W[0] < 1000 && (LP_all_W[0] != LP_all_W[1] || initScreen))
    {
      display.setTextColor(ST77XX_BLACK);
      display.println("LP (kW)");
      // display description and value in W
      display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,SCREEN_HEIGHT/3*2+19); // Text size 1 has width of 6
      display.setTextColor(ST77XX_WHITE);
      display.println("LP (W)");
    }
    else if (LP_all_W[0] != LP_all_W[1] || initScreen)
    {
      display.setTextColor(ST77XX_BLACK);
      display.println("LP (W)");
      // display description and value in kW
      display.setCursor(SCREEN_WIDTH/2-shift_k_value-shift_dot-8*6,SCREEN_HEIGHT/3*2+19); // Text size 1 has width of 6
      display.setTextColor(ST77XX_WHITE);
      display.println("LP (kW)");
    }
    if (LP_all_W[0] != LP_all_W[1] || initScreen)
      {
        display.setCursor(SCREEN_WIDTH/2+2*6,SCREEN_HEIGHT/3*2+19); // Text size 1 has width of 6

        display.println("SoC LP1"); // Charge Status
        display.setTextColor(ST77XX_WHITE);
        display.setTextSize(2);
        WriteWattValue(LP_all_W[1], SCREEN_WIDTH/2-shift_k_value-shift_dot, SCREEN_HEIGHT/3*2+29, ST77XX_BLACK);
        WriteWattValue(LP_all_W[0], SCREEN_WIDTH/2-shift_k_value-shift_dot, SCREEN_HEIGHT/3*2+29, ST77XX_WHITE);
        LP_all_W[1] = LP_all_W[0];
      }

    if (LP1_SOC[0] != LP1_SOC[1] || initScreen)
    { 
      display.setTextSize(2);
      if (LP1_SOC[1] < 10)
      {
        display.setCursor(SCREEN_WIDTH-2*12,SCREEN_HEIGHT/3*2+29);
        display.setTextColor(ST77XX_BLACK);
        display.print(String(LP1_SOC[1])+"%");
      }
      else if (LP1_SOC[1] < 100)
      {
        display.setCursor(SCREEN_WIDTH-3*12,SCREEN_HEIGHT/3*2+29);
        display.setTextColor(ST77XX_BLACK);
        display.print(String(LP1_SOC[1])+"%");
      }
      else
      {
        display.setCursor(SCREEN_WIDTH-4*12,SCREEN_HEIGHT/3*2+29);
        display.setTextColor(ST77XX_BLACK);
        display.print(String(LP1_SOC[1])+"%");
      }
      if (LP1_SOC[0] < 10)
      {
        display.setCursor(SCREEN_WIDTH-2*12,SCREEN_HEIGHT/3*2+29);
        display.setTextColor(ST77XX_WHITE);
        display.print(String(LP1_SOC[0])+"%");
        drawBar(SCREEN_HEIGHT-6, LP1_SOC[0]);
      }
      else if (LP1_SOC[0] < 100)
      {
        display.setCursor(SCREEN_WIDTH-3*12,SCREEN_HEIGHT/3*2+29);
        display.setTextColor(ST77XX_WHITE);
        display.print(String(LP1_SOC[0])+"%");
        drawBar(SCREEN_HEIGHT-6, LP1_SOC[0]);
      }
      else
      {
        display.setCursor(SCREEN_WIDTH-4*12,SCREEN_HEIGHT/3*2+29);
        display.setTextColor(ST77XX_WHITE);
        display.print(String(LP1_SOC[0])+"%");
        drawBar(SCREEN_HEIGHT-6, LP1_SOC[0]);
      }
      LP1_SOC[1] = LP1_SOC[0];
    }

  // drawing if energy is imported or exported
  if (initScreen)
   { 
    // drawing the Power Symbol
    display.drawBitmap(SCREEN_WIDTH/2-20-2, 37, blitz, 8, 10, ST77XX_YELLOW);
    // drawing the house
    display.drawBitmap(SCREEN_WIDTH/2-20+10+8, 37, haus2, 16, 10, ST77XX_WHITE);
  
  }

  // drawing the arrow (from or to house)
  if (EVU_dir[0] > 0 && (EVU_dir[0] != EVU_dir[1] || initScreen))
  {
    display.drawBitmap(SCREEN_WIDTH/2-20+7, 37, arrow_left, 8, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-20+7, 37, arrow_right, 8, 10, ST77XX_RED);
    EVU_dir[1] = EVU_dir[0];
  }
  else if (EVU_dir[0] != EVU_dir[1] || initScreen)
  {
    display.drawBitmap(SCREEN_WIDTH/2-20+7, 37, arrow_right, 8, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-20+7, 37, arrow_left, 8, 10, ST77XX_GREEN);
    EVU_dir[1] = EVU_dir[0];
  }

  // drawing the battery (in/out)
  if (HB_W[0] == 0)
  {
    display.drawBitmap(SCREEN_WIDTH/2-20+10+21, 37, arrow_left, 8, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-20+10+21, 37, arrow_right, 8, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_BLACK);
  }
  else if (HB_dir > 0)
  {
    display.drawBitmap(SCREEN_WIDTH/2-20+10+21, 37, arrow_left, 8, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-20+10+21, 37, arrow_right, 8, 10, ST77XX_GREEN);
    if (HB_SOC[0] <= batteryRedPercent)
      {
        display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_RED);
      }
    else if (HB_SOC[0] > batteryRedPercent && HB_SOC[0] <= batteryYellowPercent)
      {
        display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_YELLOW);
      }
    else
        display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_GREEN);
  }
  else if (HB_dir < 0)
  {
    display.drawBitmap(SCREEN_WIDTH/2-20+10+21, 37, arrow_right, 8, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-20+10+21, 37, arrow_left, 8, 10, ST77XX_RED);
    if (HB_SOC[0] <= batteryRedPercent)
      {
        display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_RED);
      }
    else if (HB_SOC[0] > batteryRedPercent && HB_SOC[0] <= batteryYellowPercent)
      {
        display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_YELLOW);
      }
    else
        display.drawBitmap(SCREEN_WIDTH/2-20+10+32, 37, battery, 8, 10, ST77XX_GREEN);
  }  

  // CHARGE STATE
  //
  // drawing the status of the charging station PLUGGED - UNPLUGGED - CHARGING (charging is also plugged!)
  if(LP1_PlugStat[0]==true && ((LP1_PlugStat[0] != LP1_PlugStat[1]) || initScreen))
  {
    // charging, drawing plugged car only - IMPORTANT BUG POSSIBLE POWER SYMBOL COULD STAY, MIGHT NEED ST77XX_BLACK SQUARE DRAWING
    display.drawBitmap(SCREEN_WIDTH/2-14, SCREEN_HEIGHT/3*2+5, unplugged, 20, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-14, SCREEN_HEIGHT/3*2+5, plugged, 24, 10, ST77XX_WHITE);
    LP1_PlugStat[1] = LP1_PlugStat[0];
  }
  else if ((LP1_PlugStat[0] != LP1_PlugStat[1]) || initScreen)
  {
    WriteLog("UNPLUGGED DRAWING");
    // charging, drawing unplugged car only - IMPORTANT BUG POSSIBLE POWER SYMBOL COULD STAY, MIGHT NEED ST77XX_BLACK SQUARE DRAWING
    display.drawBitmap(SCREEN_WIDTH/2-14, SCREEN_HEIGHT/3*2+5, plugged, 24, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-14, SCREEN_HEIGHT/3*2+5, unplugged, 20, 10, ST77XX_WHITE);
    LP1_PlugStat[1] = LP1_PlugStat[0];
  }
  
  if(LP1_IsCharging)
  {
    // charging, drawing plugged car plus Power symbol
    display.drawBitmap(SCREEN_WIDTH/2-14, SCREEN_HEIGHT/3*2+5, plugged, 24, 10, ST77XX_WHITE);
    display.drawBitmap(SCREEN_WIDTH/2-14+24+2, SCREEN_HEIGHT/3*2+5, blitz, 8, 10, ST77XX_YELLOW);
  }
  else
  {
    // charging, drawing plugged car plus Power symbol
    //display.drawBitmap(SCREEN_WIDTH/2-14, SCREEN_HEIGHT/3*2-4, plugged, 24, 10, ST77XX_BLACK);
    display.drawBitmap(SCREEN_WIDTH/2-14+24+2, SCREEN_HEIGHT/3*2+5, blitz, 8, 10, ST77XX_BLACK);
  }

 initScreen = false; 
}

// ------------------------------------------------
//   SETUP running once at the beginning
// ------------------------------------------------
//   Initialize  Serial, WiFi and Siplay

void setup() 
{
  Serial.begin(115200);

  while (!Serial) { // wait for serial port to connect. 
    ; 
  }
  WriteLog("openWB Display Init");

    // Use this initializer if using a 1.8" TFT screen:
  display.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  display.invertDisplay(false);
  display.setRotation(2);
  // Clear the buffer
  display.setCursor(0,0);
  display.fillScreen(ST77XX_BLACK);
  
  display.println("Waiting for WiFi");
  WiFi.mode(WIFI_STA);                             // connect to AP
  WiFi.begin(ssid, pass);                          // set WiFi connections params
  WiFi.hostname(hostname);
 
  // Connecting
  int timout = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    display.setTextSize(1);
    display.print("O");
    timout++;
    if  (timout > 20)                 // couldn'T connect to WiFi within timeout. No WiFi. Need to add better handling
    {
      WriteLog("");
      WriteDisplayNewText("Error connecting to WiFi. Exiting.");
      display.println("Error connecting to WiFi. Exiting.");
      break;
    }
  }
 
  if (WiFi.status() == WL_CONNECTED)
  {
    WriteLog("");
    WriteLog("Connected to WiFi:");
    display.println("Connected to WiFi:");
    Serial.println(WiFi.localIP());
    display.println(WiFi.localIP());
  }

  MDNS.begin(hostname);               // Start mDNS 
  server.on("/", HandleRoot);         // Call function if root is called
  
  httpUpdater.setup(&server);         // Updater
  server.begin();                     // start HTTP server
  WriteLog("HTTP server started");
   
  MQTTClient.setServer(MQTT_Broker,MQTT_Broker_Port);
  MQTTClient.setCallback(MQTTCallback);
  lastReconnectAttempt = 0;
  MQTTReconnect;
  
  WriteLog("Exiting Setup, starting main loop");

  display.setCursor(0,0);
  display.fillScreen(ST77XX_BLACK);
  UpdateDisplay();
}

// ------------------------------------------------
//   MAIN LOOP RUNNING all the time
// ------------------------------------------------
void loop() 
{
  unsigned long currentMillis = millis(); 
  if (!MQTTClient.connected())      // non blocking MQTT reconnect sequence
    {
        long now = millis();
        if (now - lastReconnectAttempt > 5000) 
        {
          lastReconnectAttempt = now;
          WriteLog("Attempting to reconnect MQTT");
          if (MQTTReconnect()) 
          {
              lastReconnectAttempt = 0;
          }
        }
    }
    else                            // MQTT is connected, lets send some data
    { 
        // do things
    }
  if (ErrorWasActive)
  {
    initScreen = true;
    ErrorWasActive = false;
  }
  else if (millis()-lastMQTTDataReceived > MaxDataAge && !ErrorWasActive)
  {
    display.setCursor(0,0);
    display.fillScreen(ST77XX_BLACK);
    display.setTextSize(3);
    display.setCursor(0,0);
    display.println("Error");
    display.println("no data");
    ErrorWasActive = true;
    delay(2000);
  }

  MQTTClient.loop();                    // handle MQTT client & subscription. Display logic is subscription event triggered and can be found in the callback function.
  server.handleClient();                // handle webserver requests
  MDNS.update();                        // handle mDNS requests

  //if (currentMillis - previousMillisTimer >= interval) 
  if (NewData)
    { 
    previousMillisTimer = currentMillis; 
    //display.startWrite();
    UpdateDisplay();
    //display.endWrite();
    NewData = false;
    }
}

void drawBar (int height, int percent)
{
  int redpos = SCREEN_WIDTH*batteryRedPercent/100;
  int yellowpos = SCREEN_WIDTH*batteryYellowPercent/100;
  int pos = SCREEN_WIDTH*percent/100;
  int npos = SCREEN_WIDTH*(100-percent)/100;
  if (pos <= redpos)
    {
      display.fillRect(0, height, pos, 5, ST77XX_RED);  
    }
  else if (pos > redpos && pos <= yellowpos)
    {
      display.fillRect(0, height, redpos, 5, ST77XX_RED);  
      display.fillRect(redpos, height, pos-redpos, 5, ST77XX_YELLOW);  
    }
  else if (pos > redpos && pos > yellowpos)
    {
      display.fillRect(0, height, redpos, 5, ST77XX_RED);  
      display.fillRect(redpos, height, yellowpos-redpos, 5, ST77XX_YELLOW);
      display.fillRect(yellowpos, height, pos-yellowpos, 5, ST77XX_GREEN);
    }
  display.fillRect(pos, height, npos, 5, ST77XX_BLACK);
}