#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Stack.h>
#include <cstdio>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#include "password.h"

#define GyroController

const char* ssid = SSID;
const char* password = PASSWORD;
const char* udp_address = "192.168.43.237";
const int udp_port = 3333;
bool wifi_connected = false;

WiFiUDP udp;
MPU9250 IMU;

int last_time=0;
float rotate=0;
int last_region=-1;

void connect_WiFi(){
    WiFi.disconnect(true);
    WiFi.begin(ssid,password);

    int try_connect_count = 0;
    while(WiFi.status() != WL_CONNECTED){
        delay(1000);
        M5.Lcd.println("WiFi connecting...");
        if((++try_connect_count) % 5 == 0){
            WiFi.disconnect(true);
            WiFi.begin(ssid,password);
        }
    }
    M5.Lcd.println("WiFi Connected!");
}

#ifdef ButtonController
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
#endif

#ifdef GyroController
void acc_sensor_controller(){
    if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
        IMU.readAccelData(IMU.accelCount);
        IMU.getAres();                                // IMU.aresが更新される ares : Scale resolutions per LSB for the sensors
        IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
        IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
        IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];
        IMU.readGyroData(IMU.gyroCount);              // Read the x/y/z adc values
        IMU.getGres();

        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
        IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
        IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;
    }
    IMU.updateTime();

    int time_diff = millis() - last_time;
    rotate += (int)(time_diff * IMU.gz / 1000.0);

    int sx = 160, sy = 120;
    float rad = rotate / 180.0 * PI;
    int gx = sx - 100 * sin(rad);
    int gy = sy - 100 * cos(rad);
    M5.Lcd.drawLine(sx,sy,gx,gy,WHITE);
    last_time = millis();

    int region = (gx < sx) ? 0 : 1; 
    M5.Lcd.print("Region : "); M5.Lcd.println(region);

    if (last_region != region){
        M5.Lcd.println("Send Packet");
        udp.beginPacket(udp_address,udp_port);
        udp.print(region);
        udp.endPacket();
        last_region = region;
    }
}

#endif

void setup() {
    Serial.begin(115200);
    delay(1000);
    M5.begin();
    Wire.begin();
    Serial.println("Start");

    M5.Lcd.println("This is Robot Controller");

    delay(200);
    // WIFI接続完了を待つ
    connect_WiFi();

    // Initialize UDP state
    udp.begin(udp_port);

    // センサーの初期化
    IMU.calibrateMPU9250(IMU.gyroBias,IMU.accelBias);
    IMU.initMPU9250();
    last_time = millis();
}

void loop() {
    // put your main code here, to run repeatedly:
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0,0);

    /* ボタンを使ったコントローラ */
    //button_controller();

    /* 加速度センサを使ったコントローラ */
    acc_sensor_controller();
    M5.update();
    delay(200);
}