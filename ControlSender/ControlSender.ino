#include<esp_now.h>
#include<WiFi.h>

String myString;

////////

uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x99, 0xB9, 0x84};

typedef struct controlMessage {
  float steer;
  float throttle;
} controlMessage;

controlMessage cData;

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

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()>0){
    myString = Serial.readStringUntil('\n');
//    Serial.print("R ");

    int i1 = myString.indexOf(',');
    int i2 = myString.indexOf(']');
    float num1 = myString.substring(1,i1).toFloat();
    float num2 = myString.substring(i1+2,i2).toFloat();

    cData.steer = num1;
    cData.throttle = num2;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &cData, sizeof(cData));


    if (result == ESP_OK){
      Serial.println("Sending Confirmed");
    } else{
      Serial.println("Sending error");
    }

//    Serial.print(num1);
//    Serial.print(' ');
//    Serial.println(num2);
  }

}
