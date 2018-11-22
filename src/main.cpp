//#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <M5Stack.h>

#include "password.h"

const char* ssid = SSID;
const char* password = PASSWORD;

HTTPClient http;
WiFiMulti wifi;

void setup() {
    wifi.addAP(ssid,password);
    Serial.begin(115200);
    M5.begin();

    // WIFI接続完了を待つ
    while(wifi.run() != WL_CONNECTED){
        delay(500);
        M5.Lcd.printf("WiFi Connecting...");
    }
    M5.Lcd.println("WiFi Connected!");
}

void loop() {
    // put your main code here, to run repeatedly:
}
