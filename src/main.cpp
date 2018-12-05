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
const char* udp_address = "192.168.100.121";
const int udp_port = 3333;
bool wifi_connected = false;

WiFiUDP udp;
MPU9250 IMU;

int last_time=0;
float rotatex=0;
float rotatey=0;
float rotatez=0;
int last_region=-1;
int gxlast = 0, gylast = 0;

/*
  M_OJIGI         = 0x0B80, // お辞儀
  M_HOME_POSITION = 0x1380, // ホームhポジション
                            // WALK
  M_TO_LEFT       = 0x3380, // To left
  M_TO_RIGHT      = 0x3B80, // To right
  M_TURN_LEFT     = 0x4380, // Turn left
  M_TURN_RIGHT    = 0x4B80, // Turn right
  M_GET_UP_U      = 0x5380, // 起き上がり うつぶせ
  M_GET_UP_A      = 0x5B80, // M011  起き上がり 仰向け
  M_PUNCHL        = 0x6380, // M012  横パンチ左
  M_PUNCHR        = 0x6B80, // M013  横パンチ右
  M_UTUBUSE       = 0x7380, // M014  うつ伏せ
  M_AOMUKE        = 0x7B80, // M015  仰向け
  M_WAVE_HAND     = 0x8380, // M016  手を振る
*/

const int motion_num = 14;
int motion_index = 0;
String motions[] = {"Bow","Home Position","Walk","To left","To right","Turn left",
                    "Turn right","GetUp A","GetUp B","Punch left","Punch right","Prone","Lie","Wave"};

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
        Serial.println(motion_index);
        char s = 'a' + motion_index;
        udp.print(s);
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
    if(gx != gxlast || gy != gylast){
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.drawLine(sx,sy,gx,gy,WHITE);
        gxlast = gx;
        gylast = gy;
    }
    last_time = millis();

    int region = (gx < sx) ? 0 : 1; 
    if(gx < sx && gy < sy){
        region = 0;
    }else if(gx < sx && gy >= sy){
        region = 1;
    }else if(gx >= sx && gy < sy){
        region = 2;
    }else{
        region = 3;
    }

    if (last_region != region){
        M5.Lcd.setCursor(0,0);
        M5.Lcd.println("Send Packet");
        udp.beginPacket(udp_address,udp_port);
        char sendm = 'A' + region;
        Serial.println(sendm);
        udp.print(sendm);
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