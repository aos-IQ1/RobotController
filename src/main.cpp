#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Stack.h>
#include <cstdio>

#include "password.h"

const char* ssid = SSID;
const char* password = PASSWORD;
const char* udp_address = "192.168.120.116";
const int udp_port = 3333;
bool wifi_connected = false;

WiFiUDP udp;

void connect_WiFi(){
    WiFi.disconnect(true);
    WiFi.begin(ssid,password);

    while(WiFi.status() != WL_CONNECTED){
        delay(1000);
        M5.Lcd.println("WiFi connecting...");
    }
    M5.Lcd.println("WiFi Connected!");
}

void button_controller(){
    udp.beginPacket(udp_address,udp_port);
    if(M5.BtnA.wasPressed()){
        udp.print("A");
        Serial.println("ButtonA is Pressed");
    }
    if(M5.BtnB.wasPressed()){
        udp.print("B");
        Serial.println("ButtonB is Pressed");
    }
    if(M5.BtnC.wasPressed()){
        udp.print("C");
        Serial.println("ButtonC is Pressed");
    }
    udp.endPacket();
}


void setup() {
    Serial.begin(115200);
    M5.begin();
    Serial.println("Start");

    // WIFI接続完了を待つ
    connect_WiFi();

    // Initialize UDP state
    udp.begin(udp_port);
}

void loop() {
    // put your main code here, to run repeatedly:
    M5.update();
    M5.Lcd.println("Looping");

    /* ボタンを使ったコントローラ */
    button_controller();

    /* 加速度センサを使ったコントローラ */

    delay(200);
}
