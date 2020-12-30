// Standard includes
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

// Include the display Library
#include <U8x8lib.h>        // https://github.com/olikraus/u8g2

// Include the accelerometer Library
#include <TinyGPS++.h>      // https://github.com/mikalhart/TinyGPSPlus

// Wifi credentials held seperately
#include "..\WifiCredentials.h"

// Serial port usage
#define USB_SERIAL_PORT Serial
#define GPS_SERIAL_PORT Serial2

// Display parameters
U8X8_SSD1306_128X32_UNIVISION_HW_I2C display;

// GPS engine
TinyGPSPlus gps;

// Wifi config
const int serverPort = 8888;
WiFiServer server(serverPort);
const int MAX_CLIENTS = 1;
WiFiClient client[MAX_CLIENTS];

// Ethernet config
const IPAddress local_IP(192, 168, 1, 8);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress primaryDNS(8, 8, 8, 8);
const IPAddress secondaryDNS(8, 8, 4, 4);

// Other vars
int32_t const BUFFER_SIZE(1024);
static uint8_t dataBuffer[BUFFER_SIZE];      
static char strBuffer[BUFFER_SIZE];      

void scanNetworks() {

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Scanning network...");
  int32_t n = WiFi.scanNetworks();
  Serial.print(n); Serial.println(" network(s) found");
  for (int32_t i = 0; i < n; i++) {
    Serial.println(WiFi.SSID(i));
  }
  Serial.println();
}

void setup() {

  // Initialise the serial port
  USB_SERIAL_PORT.begin(115200);

  // Initialize GPS
  GPS_SERIAL_PORT.begin(9600, SERIAL_8N1);

  // Initialize I2C
  Wire.begin();

  // Initialise display
  display.begin();
  display.setPowerSave(0);
  display.setContrast(32);
  display.clear();
  display.setFont(u8x8_font_amstrad_cpc_extended_f);           
  display.drawString(0,0,"GPS Passthrough");
  display.drawString(0,1," Wifi...");
  display.drawString(0,2," GPS...");

  USB_SERIAL_PORT.println("GPS Passthrough");
  scanNetworks();

  // Init wifi connection
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    USB_SERIAL_PORT.println("Failed to configure network settings");
  }
  WiFi.begin(wifiSSID, wifiPassword);
  USB_SERIAL_PORT.print("Connecting to wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    USB_SERIAL_PORT.print('.');
    delay(1000);
  }
  USB_SERIAL_PORT.println();
  USB_SERIAL_PORT.println("Connected to wifi.");
  USB_SERIAL_PORT.print("IP addr: "); USB_SERIAL_PORT.println(WiFi.localIP());
  display.drawString(0, 1, WiFi.localIP().toString().c_str());

  // Start server
  server.begin();
  server.setNoDelay(true);  // No message combining

  delay(2000);
}

void loop() {

  // Check if any client is trying to connect
  if (server.hasClient()) {
    USB_SERIAL_PORT.println("Client connecting...");

    for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
      // Find free/disconnected spot
      if (!client[i] || !client[i].connected()) {
        if (client[i]) 
          client[i].stop();
        client[i] = server.available();
        USB_SERIAL_PORT.println("Accepted client.");
      }
    }
    // No remaining free/disconnected spots so reject any remaining requests to connect
    while (server.hasClient()) {
      server.available().stop();
      USB_SERIAL_PORT.println("Rejected client.");
    }
  }

  // For each client, take any data they are sending and forward to the serial port
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (client[i]) {
      int32_t bytesRead(0);
      while (client[i].available() && (bytesRead < BUFFER_SIZE)) {
        dataBuffer[bytesRead++] = client[i].read();
      }

      GPS_SERIAL_PORT.write(dataBuffer, bytesRead);
    }
  }

  // If the serial port has any data, then forward it to each client
  if (GPS_SERIAL_PORT.available()) {
    String str;
    while (GPS_SERIAL_PORT.available() > 0) {
      int c = GPS_SERIAL_PORT.read();
      gps.encode(c);
      USB_SERIAL_PORT.write(c);
      str += (char)c; // read char from UART(num)
    }

    // Now send to WiFi
    for(uint8_t i = 0; i < MAX_CLIENTS; i++) {
      if(client[i]) {
        client[i].write(str.c_str());
      }                    
    }        
  }

  static uint32_t gpsTimer(0);

  if((millis()-gpsTimer) > 500) { 

    if (gps.location.isUpdated()) {
      sprintf(strBuffer, "%+02.3f %+03.3f", gps.location.lat(), gps.location.lng());   // "+90.000 +180.000"
      display.drawString(0,0,strBuffer);
    }

    if (gps.time.isUpdated()) {
      sprintf(strBuffer, "%02d:%02d:%02d %02d%02d%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year()%100);   // "00:00:00 010203"
      display.drawString(0,1,strBuffer);
    }

    if (gps.satellites.isUpdated() || gps.hdop.isUpdated()) {
      sprintf(strBuffer, "SVS%02d HDOP%.1f", gps.satellites.value(), gps.hdop.value()/100.0);   // "SVS99 HDOP99.9"
      display.drawString(0,2,strBuffer);
    }

    sprintf(strBuffer, "P%3d F%3d X%3d", gps.passedChecksum() % 1000, gps.failedChecksum() % 1000, gps.sentencesWithFix() % 1000); // "P999 F000 X000"
    display.drawString(0,3,strBuffer);

    gpsTimer = millis();  
  }
}

