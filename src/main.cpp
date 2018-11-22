#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Stack.h>

#include "password.h"

const char* ssid = SSID;
const char* password = PASSWORD;
const char* udp_address = "192.168.0.255";
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


void setup() {
    Serial.begin(115200);
    M5.begin();

    // WIFI接続完了を待つ
    connect_WiFi();

    // Initialize UDP state
    udp.begin(WiFi.localIP(),udp_port);
}

void loop() {
    // put your main code here, to run repeatedly:

    udp.beginPacket();
    udp.print("poipoi~");
    udp.endPacket();
    delay(2000);
}
