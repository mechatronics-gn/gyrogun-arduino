#include "ESP8266.h"
#include <SoftwareSerial.h>
#include "secrets.h"


#define ESP_RX 2
#define ESP_TX 3
#define SWITCH 4

SoftwareSerial esp_serial(ESP_TX, ESP_RX);
ESP8266 wifi(esp_serial);

void setup() {
  Serial.begin(9600);

  pinMode(SWITCH, INPUT_PULLUP);
  
  wifi.setOprToStationSoftAP();

  wifi.joinAP(MECHANET_SSID, MECHANET_PW);
  Serial.println(wifi.getLocalIP().c_str());
  
  wifi.disableMUX();

  wifi.createTCP(HOST_IP, HOST_PORT);
}

int switch_state = 0;

int count = 0;


void loop() {  
  bool switch_was_clicked = false;
  
  int switch_val = digitalRead(SWITCH);
  if (switch_val == 0 && switch_state == 1) {
    switch_was_clicked = true;
  }
  switch_val = switch_state;

  // 대충 자이로 받아오는 코드

  float yaw = 0.0, pitch = 0.0, roll = 0.0;

  uint8_t packet[16] = {};

  uint32_t message_type = 0;
  if (switch_was_clicked) message_type = 1;
  message_type = count++;

  packet[0] = message_type >> 24 % 256;
  packet[1] = message_type >> 16 % 256;
  packet[2] = message_type >> 8 % 256;
  packet[3] = message_type >> 0 % 256;

  uint32_t yaw_uint = 0;
  memcpy(&yaw_uint, &yaw, sizeof(uint32_t));

  packet[4] = yaw_uint >> 24 % 256;
  packet[5] = yaw_uint >> 16 % 256;
  packet[6] = yaw_uint >> 8 % 256;
  packet[7] = yaw_uint >> 0 % 256;

  uint32_t pitch_uint = 0;
  memcpy(&pitch_uint, &pitch, sizeof(uint32_t));

  packet[8] = pitch_uint >> 24 % 256;
  packet[9] = pitch_uint >> 16 % 256;
  packet[10] = pitch_uint >> 8 % 256;
  packet[11] = pitch_uint >> 0 % 256;

  uint32_t roll_uint = 0;
  memcpy(&roll_uint, &roll, sizeof(uint32_t));

  packet[12] = roll_uint >> 24 % 256;
  packet[13] = roll_uint >> 16 % 256;
  packet[14] = roll_uint >> 8 % 256;
  packet[15] = roll_uint >> 0 % 256;
    
  if(!wifi.send(packet, 16)) {
    Serial.println("send failure, trying to reconnect");
    while(true) {
      if(wifi.createTCP(HOST_IP, HOST_PORT)) {
        Serial.println("reconnection successful");
        break;
      } else {
        Serial.println("reconnection failed, will try again in 3 secs...");
        delay(3000);
      }
    }
  }

  delay(50);
}
