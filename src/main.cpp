#include <Arduino.h>
#include <DFRobot_SIM808.h>
#include <HardwareSerial.h>
#include <WiFiManager.h> 
#include <HTTPClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#define _CUSTOM
HardwareSerial serial(2);
DFRobot_SIM808 sim808(&serial);

//INSERT INTO `coordinates` (`trackingID`, `longitude`, `latitude`, `date_created`, `status`) VALUES ('6', '0.2', '0.2', '2023-10-30 03:40:50.000000', '1')
WiFiClient client;                 


//const char* serverAddy = "http://192.168.4.2/esp32_receiver/index.php";
//const char* serverAddy = "http://192.168.4.2/esp32_receiver/index.php";

String serverAddy;

void setup() {
  
  Serial.begin(115200);
  serial.begin(9600, SERIAL_8N1, 16, 17);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.resetSettings();
  WiFiManagerParameter param("sv", "server address", "http://192.168.4.2/esp32_receiver/index.php", 64);
  wm.addParameter(&param);

  wm.autoConnect("ESP32_ap", "admin123");
  
  serverAddy = String(param.getValue());

  while(!sim808.init()){
    Serial.println("[ERROR] Sim init!");
    delay(350);
  }
  
  if( sim808.attachGPS())
      Serial.println("Open the GPS power success");
  else 
      Serial.println("Open the GPS power failure");
  
  

  
  
}


void handleGPS(){
  if (sim808.getGPS()) {
    HTTPClient http;
    http.begin(serverAddy.c_str());
    http.addHeader("Content-Type", "application/json");

    String lat(sim808.GPSdata.lat, 6);
    String lon(sim808.GPSdata.lon, 6);
    String payload = "{\"latitude\" : " + lat + ",\"longitude\" : " + lon + "}";
    Serial.print("SQL: ");
    Serial.println(payload);
    Serial.print("Result: ");
    Serial.println(http.POST(payload));

    //sim808.detachGPS();
    delay(10000); // send coordinates every 10 seconds
  }
}

void loop() {



  handleGPS();
  
  
}

