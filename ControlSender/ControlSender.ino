#include<esp_now.h>
#include<WiFi.h>

String myString;

////////

uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x99, 0xB9, 0x84};

typedef struct __attribute__((packed)) controlMessage {
  float steer;
  float throttle;
  bool manualDriving;
  int16_t DPadSumX;
  int16_t DPadSumY;
  int16_t BumperSum;
  bool A = false;
  int8_t endSig;
}controlMessage;


typedef struct __attribute__((packed)) returnMessage {
  int8_t startSig = 69;
  float steer;
  float throttle;
  bool manualDriving = true;
  int16_t DPadSumX;
  int16_t DPadSumY;
  int16_t BumperSum = 43;
  bool A = false;
  int8_t endSig = 96;
}returnMessage;



returnMessage getControlMessage() {
  controlMessage cData;
  returnMessage retData;
  int8_t aByte;
  byte messageBuffer[sizeof(cData)];
  
  aByte = Serial.read();
  if (aByte == 69){
    Serial.readBytes(messageBuffer, sizeof(messageBuffer));
    memcpy(&cData, messageBuffer, sizeof(messageBuffer));
    if (cData.endSig == 96){
      retData.steer = cData.steer;
      retData.throttle = cData.throttle;
      retData.manualDriving = cData.manualDriving;
      retData.DPadSumX = cData.DPadSumX;
      retData.DPadSumY = cData.DPadSumY;
      retData.BumperSum = cData.BumperSum;
      retData.A = cData.A;
      return retData;
    } else{
      retData.endSig = 95;
      return retData;
    }
  }
  else{
    retData.endSig = 95;
    return retData;
  }
}






esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



////////



void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

////////
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
//////////

}

//byte messageBuffer[sizeof(cData)];
int8_t byteFinder;
int8_t startByte = 69;

returnMessage rData;

void loop() {
  // put your main code here, to run repeatedly:
  bool isData = false;
  rData = getControlMessage();
  if (rData.endSig == 96) {
    Serial.write((byte*)&rData, sizeof(rData));
  }
  
  
//  Serial.readBytes(messageBuffer, sizeof(cData));
//  memcpy(&cData, &messageBuffer, sizeof(cData));


//  byteFinder = 0;
//  byteFinder = Serial.read();
//  if (byteFinder == 69){
//    Serial.readBytes(messageBuffer, sizeof(messageBuffer));
//    memcpy(&cData, messageBuffer, sizeof(cData));
//    rData.steer = cData.steer;
//    rData.throttle = cData.throttle;
//    rData.manualDriving = cData.manualDriving;
//    rData.DPadSumX = cData.DPadSumX;
//    rData.DPadSumY = cData.DPadSumY;
//    rData.BumperSum = cData.BumperSum;
//    rData.A = cData.A;
////    Serial.write((byte*)startByte, sizeof(startByte));
////    cData.startSig = 69;
//    Serial.write((byte*)&rData, sizeof(rData));
////    delay(1000);
//  }

  
  
//  Serial.write(messageBuffer, sizeof(messageBuffer));
  
  


//    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &cData, sizeof(cData));


//    if (result == ESP_OK){
//      Serial.println("Sending Confirmed");
//    } else{
//      Serial.println("Sending error");
//    }

//    Serial.print(num1);
//    Serial.print(' ');
//    Serial.println(num2);

}
