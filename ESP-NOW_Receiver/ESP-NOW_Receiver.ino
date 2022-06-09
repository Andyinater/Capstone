#include<esp_now.h>
#include<WiFi.h>
#include <ESP32Servo.h>

typedef struct controlMessage {
  float steer;
  float throttle;
} controlMessage;

controlMessage cData;

Servo throttleServo;
Servo steerServo;

int throttlePin = 25;
int steerPin = 26;
int throttleMin = 500; // 1000 for steer, 500 for throttle
int throttleMax = 1500; // 3000 for steer. 1500 for throttle
int steerMin = 1000;
int steerMax = 3000;


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
  memcpy(&cData, incomingData, sizeof(cData));
//  Serial.print("Data received: ");
//  Serial.println(len);
//  Serial.print("Steer Value:");
//  Serial.print(cData.steer);
//  Serial.print("     Throttle Value:");
//  Serial.println(cData.throttle);

  throttleServo.write(cData.throttle);
  steerServo.write(cData.steer);
  
  // necessary to not upset watchdog
  delay(1);
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
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  throttleServo.setPeriodHertz(50);    // standard 50 hz servoMax
  steerServo.setTimerWidth(30);
  throttleServo.attach(throttlePin, throttleMin, throttleMax);
  steerServo.attach(steerPin, steerMin, steerMax);

}

void loop() {
  // put your main code here, to run repeatedly:

}
