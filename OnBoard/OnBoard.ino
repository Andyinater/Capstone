#include<esp_now.h>
#include<WiFi.h>
#include<ESP32Servo.h>
#include<MPU9250.h> //Gz for yaw, Ay for Lat, Ax for long (if positioned same as Liburdi)
#include"eeprom_utils.h"

uint8_t broadcastAddress[] = {0x84, 0xCC, 0xA8, 0x7E, 0xD0, 0x60}; //84:cc:a8:7e:d0:60

esp_now_peer_info_t peerInfo;


typedef struct __attribute__((packed)) controlMessage {
  int8_t startSig = 69;
  float steer;
  float throttle;
  bool manualDriving;
  int16_t DPadSumX;
  int16_t DPadSumY;
  int16_t BumperSum;
  bool A;
  int8_t endSig = 96;
}controlMessage;

typedef struct __attribute__((packed)) returnMessage {
  int8_t startSig = 69;
  float steer;
  float throttle;
  bool manualDriving;
  int16_t DPadSumX;
  int16_t DPadSumY;
  int16_t BumperSum;
  bool A;
  float dTime;
  float gZ;
  float aX;
  float aY;
  int8_t endSig = 96;
}returnMessage;

returnMessage rData;
controlMessage cData;

Servo throttleServo;
Servo steerServo;

int throttlePin = 25;
int steerPin = 26;
int throttleMin = 500; // 1000 for steer, 500 for throttle
int throttleMax = 1500; // 2500 for steer. 1500 for throttle
int steerMin = 1000;
int steerMax = 2500;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double lastControlTime = 0;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
  memcpy(&rData, incomingData, sizeof(returnMessage));
  lastControlTime = millis();
//  Serial.print("Data received: ");
//  Serial.println(len);
//  Serial.print("Steer Value:");
//  Serial.print(mapfloat(rData.steer,-1,1,steerMax,steerMin)); // Flip for controller sense
//  Serial.print("     Throttle Value:");
//  Serial.println(mapfloat(rData.throttle,-1,1,throttleMin,throttleMax));
  // necessary to not upset watchdog
  delay(1);
}

MPU9250 mpu;
float IMU_VALS[4] = {0, 0, 0, 0};

returnMessage createReturnData(returnMessage rrData){
  rrData.startSig = 69;
  rrData.steer = rData.steer;
  rrData.throttle = rData.throttle;
  rrData.manualDriving = rData.manualDriving;
  rrData.DPadSumX = rData.DPadSumX;
  rrData.DPadSumY = rData.DPadSumY;
  rrData.BumperSum = rData.BumperSum;
  rrData.A = rData.A;
  rrData.dTime = IMU_VALS[0];
  rrData.gZ = IMU_VALS[1];
  rrData.aX = IMU_VALS[2];
  rrData.aY = IMU_VALS[3];
  rrData.endSig = 96;
  return rrData;
}





void setup() {
  // put your setup code here, to run once:

  
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  throttleServo.setPeriodHertz(50);    // standard 50 hz servoMax
  steerServo.setTimerWidth(30);
  throttleServo.attach(throttlePin);
  steerServo.attach(steerPin);

  initIMU();

}

double startTime = -1;
double funcSteer = 0;
double t = 0;

void loop() {
  // put your main code here, to run repeatedly:
//  if (mpu.update()) {
//    static uint32_t prev_ms = millis();
//    if (millis() > prev_ms + 25) {
//      update_IMU_VALS();
//      rData = createReturnData(rData);
//      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &rData, sizeof(returnMessage));
//    }
//  }
  

  
  if (rData.manualDriving){
    throttleServo.writeMicroseconds(mapfloat(rData.throttle,-1,1,throttleMin,throttleMax));
    steerServo.writeMicroseconds(mapfloat(rData.steer,-1,1,steerMax,steerMin));

    Serial.print(rData.throttle);
    Serial.print(",  ");
    Serial.println(mapfloat(rData.steer,-1,1,steerMax,steerMin));

    if (rData.A == true) {
      if (mpu.update()) {
          static uint32_t prev_ms = millis();
          if (millis() > prev_ms + 25) {
            update_IMU_VALS();
            rData = createReturnData(rData);
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &rData, sizeof(returnMessage));
          }
        }
    }
    
  } else { 
    // programmed manouver mode
    if (rData.A){ 
      // if holding A to perform manouver
      if (startTime == -1){ 
        // if starting manouver
        startTime = millis();
      } else if (millis() - lastControlTime < 1000){ 
        // if controller hasn't disconnected in 1 second
        // Run the manouver
        // Steer = DPadSumX * sin(DPadSumY * t * 2 * pi)
        // for starters, DPadSumX = 10 == max steer amplitude
        // DPadSumY = 0 through 10, 0% to 100% of 2pi
        // steer max magnitude ranges from 0 to 1, map to servo later
        
        t = millis()/1000.0 - startTime/1000.0; // Bring to seconds
        funcSteer = (rData.DPadSumX/10.0)*sin((rData.DPadSumY/10.0)*t*2*3.14159);

       
        Serial.println(mapfloat(funcSteer,-1,1,steerMax,steerMin));
        steerServo.writeMicroseconds(mapfloat(funcSteer,-1,1,steerMax,steerMin));

        
        if (mpu.update()) {
          static uint32_t prev_ms = millis();
          if (millis() > prev_ms + 25) {
            update_IMU_VALS();
            rData = createReturnData(rData);
            rData.steer = funcSteer;
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &rData, sizeof(returnMessage));
          }
        }
        
        
      }
    } else{
      startTime = -1;
    }
    steerServo.writeMicroseconds(mapfloat(rData.steer,-1,1,steerMax,steerMin));
    throttleServo.writeMicroseconds(mapfloat(rData.throttle,-1,1,throttleMin,throttleMax));

  }

}

void calibrateIMU(){
  #if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.begin(0x80);
  #endif
//  mpu.verbose(true);
//  delay(5000);
//  mpu.calibrateAccelGyro();
//  // save to eeprom
//  saveCalibration();

  // load from eeprom
  loadCalibration();
}

void initIMU(){
  Wire.begin(32,33); // SDA SCL
//  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

  calibrateIMU();
}

void update_IMU_VALS(){
  IMU_VALS[0] = millis()/1000.0;
  IMU_VALS[1] = mpu.getGyroZ();
  IMU_VALS[2] = mpu.getAccX()*9.81;
  IMU_VALS[3] = mpu.getAccY()*9.81;

}
