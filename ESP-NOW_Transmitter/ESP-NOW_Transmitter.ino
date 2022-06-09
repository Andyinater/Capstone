#include<esp_now.h>
#include<WiFi.h>

uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x99, 0xB9, 0x84};

typedef struct controlMessage {
  float steer;
  float throttle;
} controlMessage;

controlMessage cData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.print(cData.steer);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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

}

void loop() {
  // put your main code here, to run repeatedly:

  cData.steer = millis();
  cData.throttle = cData.steer/10;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &cData, sizeof(cData));

  if (result == ESP_OK){
    Serial.println("Sending Confirmed");
  } else{
    Serial.println("Sending error");
  }

//  delay(5);
  

}
