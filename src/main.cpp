#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Stack.h>
#include <cstdio>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#include "password.h"

const char* ssid = SSID;
const char* password = PASSWORD;
const char* udp_address = "192.168.120.116";
const int udp_port = 3333;
bool wifi_connected = false;

WiFiUDP udp;

//MPU9250 IMU;

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

void acc_sensor_controller(){
    /*
    if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){
        IMU.readAccelData(IMU.accelCount);
        IMU.getAres();  // IMU.aresが更新される ares : Scale resolutions per LSB for the sensors

        IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
        IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
        IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];
    }
    IMU.updateTime();

    IMU.delt_t = millis() - IMU.count;
    M5.Lcd.print("ax = ");
    M5.Lcd.print((int)1000 * IMU.ax);
    M5.Lcd.print(" ay = ");
    M5.Lcd.print((int)1000 * IMU.ay);
    M5.Lcd.print(" az = ");
    M5.Lcd.print((int)1000 * IMU.az);
    M5.Lcd.println(" mg");

    IMU.count = millis();
    */
}

void setup() {
    Serial.begin(115200);
    M5.begin();
    Serial.println("Start");

    // WIFI接続完了を待つ
    //connect_WiFi();

    // Initialize UDP state
    //udp.begin(udp_port);

    // センサーの初期化
    //IMU.calibrateMPU9250(IMU.gyroBias,IMU.accelBias);
}

void loop() {
    // put your main code here, to run repeatedly:
    //M5.update();
    //M5.Lcd.println("Looping");

    /* ボタンを使ったコントローラ */
    //button_controller();

    /* 加速度センサを使ったコントローラ */
    //acc_sensor_controller();

    delay(300);
}
