/*


  MIT License

  Copyright (c) 2022 LydianJay (Lloyd Jay Arpilleda Edradan)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include <Arduino.h>
#include <DFRobot_SIM808.h>
#include <HardwareSerial.h>
#include <WiFiManager.h> 
#include <HTTPClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
HardwareSerial serial(2);
DFRobot_SIM808 sim808(&serial);


WiFiClient client;                 

String serverAddy;

constexpr uint8_t EVNT_NO_GSM = 1, EVNT_NO_WIFI = 2, ENVT_GOOD = 0;
uint8_t evnt = EVNT_NO_GSM;
constexpr uint8_t statePIN = 19;

TaskHandle_t t1;

void blinkLED(void* vParams){
  uint8_t val = HIGH;

  uint8_t e = *(uint8_t*)vParams;
  while (e != ENVT_GOOD) {
    vTaskDelay( 1 / portTICK_PERIOD_MS );
    e = *(uint8_t*)vParams;
    switch (e) {
    case EVNT_NO_GSM:
      delay(100);
      digitalWrite(statePIN, val);
      val == HIGH ? val = LOW : val = HIGH;
      delay(100);
      break;
    case EVNT_NO_WIFI:
      digitalWrite(statePIN, HIGH);
      break;
    

    default:
    
      break;
    }
  }
  
  digitalWrite(statePIN, LOW);

}

void setup() {
  
  
  Serial.begin(115200);
  serial.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(statePIN, OUTPUT);
  digitalWrite(statePIN, HIGH);
  xTaskCreatePinnedToCore(blinkLED, "blink", 10000, &evnt, 1, &t1, 0);
  while(!sim808.init()){
    Serial.println("[ERROR] Sim init!");

  }
  

  evnt = EVNT_NO_WIFI;
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.resetSettings();
  WiFiManagerParameter param("sv", "server address", "http://192.168.254.1/esp32_receiver/index.php", 64);
  wm.addParameter(&param);
  
  wm.autoConnect("GSMtracker", "admin123");
  
  serverAddy = String(param.getValue());

  
  
  if( sim808.attachGPS())
      Serial.println("Open the GPS power success");
  else 
      Serial.println("Open the GPS power failure");
  
  evnt = ENVT_GOOD;
  vTaskDelete(t1);
  digitalWrite(statePIN, LOW);
  
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

