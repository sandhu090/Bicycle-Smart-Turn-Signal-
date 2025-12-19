/Users/jasonsandhu/Documents/Arduino/Bike Project /turnSignal/turnSignal.ino#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <esp_now.h> 
#include "esp_wifi.h"
#include <FastLED.h>


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// for brake lights 

#define DATA_PIN   33      // confirmed working for you
#define NUM_LEDS   8

CRGB leds[NUM_LEDS];

// the stuff for the imu
int SDAPin = 25; 
int SCLPin = 26; 
int imuSampleRate = 5000; // after how many microseconds to update

Adafruit_BNO08x bno08x;
float homeYaw = 0;

// for espnow
uint8_t receiverMac[] = {0x12, 0x34, 0x56, 0x78, 0x91, 0x01};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// imu stuff 
float _computeYaw(sh2_SensorValue_t &event) {
  float qw = event.un.gameRotationVector.real;
  float qx = event.un.gameRotationVector.i;
  float qy = event.un.gameRotationVector.j;
  float qz = event.un.gameRotationVector.k;

  float yaw = atan2f(2.0f * (qw*qz + qx*qy),
                     1.0f - 2.0f * (qy*qy + qz*qz));

  float deg = yaw * 180.0f / PI;
  if (deg < 0) deg += 360.0f;
  return deg;
}

void blinkStatus(int times) {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}


void initIMU() {
  Serial.println("Starting IMU...");

  // Start I2C
  Wire.begin(SDAPin, SCLPin);

  // Start IMU
  if (!bno08x.begin_I2C()) {
    Serial.println("❌ BNO085 NOT detected!");
    blinkStatus(1); 
    while (1);
  }

  Serial.println("✔ IMU detected!");
  blinkStatus(2); 
  
  // Request fast data
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, imuSampleRate);

  // --- FIXED CALIBRATION BLOCK ---
  Serial.println("Hold IMU still for calibration...");

  uint32_t start = millis();
  float sum = 0;
  int count = 0;

  // Give IMU time to stabilize + collect samples (3 seconds)
  while (millis() - start < 3000) {
      sh2_SensorValue_t e;

      if (bno08x.getSensorEvent(&e)) {
          if (e.sensorId == SH2_GAME_ROTATION_VECTOR) {
              float yaw = _computeYaw(e);
              sum += yaw;
              count++;
          }
      }
  }

  // Safety check: prevent divide by zero + random homeYaw
  if (count < 5) {
      Serial.println("ERROR: Not enough IMU samples! Try rebooting.");
      blinkStatus(3); 
      while (1);

  }

  homeYaw = sum / count;

  Serial.print("Home angle saved: ");
  Serial.println(homeYaw);
}



float getAngle() {
  sh2_SensorValue_t event;
  float yaw = NAN;

  // Empty ALL queued IMU events
  while (bno08x.getSensorEvent(&event)) {
    if (event.sensorId == SH2_GAME_ROTATION_VECTOR) {
      yaw = _computeYaw(event);
    }
  }

  if (isnan(yaw)) return NAN;

  float rel = yaw - homeYaw;
  if (rel < 0) rel += 360;
  if (rel >= 360) rel -= 360;

  return rel;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initLEDs() {
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);   
  FastLED.clear(true);         
}


void turnLightsRed() {
  fill_solid(leds, NUM_LEDS, CRGB(255, 0, 0));
  FastLED.show();
}


void turnLightsOff() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// can delete later not neccessary sending Callback 
void sendCall( const wifi_tx_info_t *wifi_tx_info, esp_now_send_status_t status) {
  Serial.println("sent");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}


void brakes(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {

  int status = *(int *)data;

  if(status == 1){
    turnLightsRed();
  } else {
    turnLightsOff();
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  delay(200); 
  initIMU(); 
  initLEDs();

  // setup ESPNOW
  WiFi.mode(WIFI_STA); 
  delay(100); 
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false; 

  delay(2000);

  if(esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("connected"); 
  } else {
     Serial.println("failed to print");
  }

  esp_now_register_send_cb(sendCall); 
  esp_now_register_recv_cb(brakes); 

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  float currentAngle = getAngle();
  Serial.println(currentAngle);
  esp_now_send(receiverMac, (uint8_t*)&currentAngle, sizeof(currentAngle));


  vTaskDelay(400 / portTICK_PERIOD_MS);

}

