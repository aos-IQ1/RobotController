#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Stack.h>
#include <cstdio>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#include "password.h"

#if 1
#define GyroController
#else
#define ButtonController
#endif

const char* ssid = SSID;
const char* password = PASSWORD;
const char* udp_address = "192.168.43.12";
const int udp_port = 3333;
bool wifi_connected = false;

WiFiUDP udp;
MPU9250 IMU;

int last_time=0;
float rotatex=0;
float rotatey=0;
float rotatez=0;
int last_region=-1;

/*
  M_OJIGI         = 0x0B80, // M001  お辞儀
  M_HOME_POSITION = 0x1380, // M002  ホームhポジション
  M_PRE_WALK      = 0x1B80, // M003  PreWalk
  M_WALKL         = 0x2380, // M004  WalkL
  M_WALKR         = 0x2B80, // M005  WalkR
  M_POST_WALKR    = 0x3380, // M005  PostWalkR
  M_POST_WALKL    = 0x3B80, // M005  PostWalkL
*/

const int motion_num = 7;
int motion_index = 0;
String motions[] = {"Bow","Home Position","Pre Walk","WalkL","WalkR","PostWalkR","PostWalkL"};

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
    M5.Lcd.fillScreen(BLACK);
}

#ifdef ButtonController
void button_controller(){
    udp.beginPacket(udp_address,udp_port);
    if(M5.BtnA.wasPressed()){
        motion_index = (motion_index-1+motion_num)%motion_num;
    }else if(M5.BtnC.isPressed()){
        motion_index = (motion_index+1)%motion_num;
    }else if(M5.BtnB.wasPressed()){
        udp.print(motion_index);
    }
    udp.endPacket();

    M5.Lcd.setCursor(0,0);
    for(int i=0;i<motion_num;i++){
        if(i==motion_index){
            M5.Lcd.print("-> ");
        }else{
            M5.Lcd.print("   ");
        }
        M5.Lcd.println(motions[i]);
    }
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
    rotatex += (int)(time_diff * IMU.gx / 1000.0);
    rotatey += (int)(time_diff * IMU.gy / 1000.0);
    rotatez += (int)(time_diff * IMU.gz / 1000.0);

    int sx = 160, sy = 120;
    float rad = rotatez / 180.0 * PI;
    int gx = sx - 100 * sin(rad);
    int gy = sy - 100 * cos(rad);
    M5.Lcd.drawLine(sx,sy,gx,gy,WHITE);
    last_time = millis();

    int region = (gx < sx) ? 0 : 1; 
    M5.Lcd.setCursor(0,0);
    //M5.Lcd.print("rotateX : "); M5.Lcd.println(rotatex);
    //M5.Lcd.print("rotateY : "); M5.Lcd.println(rotatey);
    //M5.Lcd.print("rotateZ : "); M5.Lcd.println(rotatez);

    
    udp.beginPacket(udp_address,udp_port);
    udp.println("G");
    udp.println(rotatex);
    udp.println(rotatey);
    udp.println(rotatez);
    udp.endPacket();

    M5.Lcd.print("Region : "); M5.Lcd.println(region);

    /* if (last_region != region){
        M5.Lcd.println("Send Packet");
        udp.beginPacket(udp_address,udp_port);
        udp.print(region);
        udp.endPacket();
        last_region = region;
    } */
}

#endif

void setup() {
    Serial.begin(115200);
    delay(1000);
    M5.begin();
    Wire.begin();
    Serial.println("Start");

    M5.Lcd.setTextFont(2);
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
    //M5.Lcd.fillScreen(BLACK);
    //Serial.println("loop");

    /* ボタンを使ったコントローラ */
    #ifdef ButtonController
    button_controller();
    #endif

    /* 加速度センサを使ったコントローラ */
    #ifdef GyroController
    acc_sensor_controller();
    #endif
    M5.update();
    #ifdef GyroController
    delay(200);
    #else
    delay(200);
    #endif
}