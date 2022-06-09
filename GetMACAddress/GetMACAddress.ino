#include<WiFi.h>

// MAC Address: 58:BF:25:99:B9:84 for cryptowatcher-cut esp

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

}

void loop() {
  // put your main code here, to run repeatedly:

}
