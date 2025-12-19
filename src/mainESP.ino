#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <esp_now.h> 
#include "esp_wifi.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//for what mode the thing is currently in
int mode = 3; 

// for turn signals lights 
bool rightOn = false; 
bool leftOn = false;
int rightPin = 13;
int leftPin = 27;

// for buttons 
int rightButton = 33;
int leftButton = 32; 

// threshold for each turn 
int rightThreshold = 35; 
int leftThreshold = 325; 

// remebr is threshold has been passed
bool rightThresholdExceeded = false;
bool leftThresholdExceeded = false;


// diff of angle that will be handled by the callback 
float angleDiff;

// pins for hall effect sensors
int hallpin1 = 21;
int hallpin2 = 22;


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
  while(1);         

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

// my thread 
void myTask(void *parameter) {

  bool rightLastState = false; 
  bool leftLastState = false;
  
  while (true) {

    // when the light right is on 
    if (rightOn) {
      rightLastState = !rightLastState;

      Serial.println("right");

      if(rightLastState) {
        digitalWrite(rightPin, HIGH);
      }else { 
        digitalWrite(rightPin, LOW);
      }
    } else{
       digitalWrite(rightPin, LOW);
    }

    if (leftOn) {
      leftLastState = !leftLastState;
      Serial.println("left");

      if(leftLastState) {
        digitalWrite(leftPin, HIGH);
      }else { 
        digitalWrite(leftPin, LOW);
      }
    } else {
      digitalWrite(leftPin, LOW);

    }
    

    vTaskDelay(250 / portTICK_PERIOD_MS); 
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// thread for brake detection
void braking(void *parameter) {


  // sending a 1 means detected 
  // sending a 0 means magnet not detected
  int state = 0; 
  
  while (true) {
    // if any of the sensors read a LOW(sencev the magnet)
    if(digitalRead(hallpin1) == LOW || digitalRead(hallpin2) == LOW ) {
      state = 1;  

      Serial.println("DETECTED");

    } else {
      state = 0; 
      Serial.println("NOT DETECTED");
    }

    esp_now_send(receiverMac, (uint8_t*)&state, sizeof(state));

    vTaskDelay(300 / portTICK_PERIOD_MS); 
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// can delete later not neccessary sending Callback 
void sendCall( const wifi_tx_info_t *wifi_tx_info, esp_now_send_status_t status) {
  Serial.println("sent");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}


void calculateAngleDiff(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {

  float otherAngle = *(float *)data;
  float currentAngle = getAngle(); 

  angleDiff = currentAngle - otherAngle;

  // Normalize to 0–360
  if (angleDiff < 0) angleDiff += 360;

  Serial.println(angleDiff);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setUpTurnSignals() {
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(leftButton, INPUT_PULLUP);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  delay(200); 
  setUpTurnSignals();
  initIMU(); 

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
  esp_now_register_recv_cb(calculateAngleDiff); 

  // setup the pin 
  pinMode(hallpin1, INPUT_PULLUP);
  pinMode(hallpin2, INPUT_PULLUP);

  xTaskCreate(myTask, "myTask", 2048, NULL, 1, NULL); 
  xTaskCreate(braking, "myTask2", 2048, NULL, 1, NULL); 

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {



  // write code for what mode it is currently in 
  // if right button is pressed 
    // remember pin is pulled up so low means it has been pressed 
  if (digitalRead(rightButton) == LOW) {

    // have to wait for release other wise to fast 
    while (digitalRead(rightButton) == LOW) { 
      vTaskDelay(5 / portTICK_PERIOD_MS); 
    }

    /// if the right light was not on turn on right and left off 
    if(rightOn == false) {
      rightOn = true; 
      leftOn = false;
      leftThresholdExceeded = false;
      // if it was on turn it off 
    } else {
      rightOn = false;
      rightThresholdExceeded = false;
    }
  }

  if (digitalRead(leftButton) == LOW) {

    // have to wait for release other wise to fast 
    while (digitalRead(leftButton) == LOW) { 
      vTaskDelay(5 / portTICK_PERIOD_MS); 
    }

    /// if the left light was not on turn on right and left off 
    if(leftOn == false) {
      leftOn = true; 
      rightOn = false;
      rightThresholdExceeded = false;
      // if it was on turn it off 
    } else {
      leftOn = false;
      leftThresholdExceeded = false;
    }
  }







  // if the light is on then check if the angle has been exceeded
    // mark it as has been exeeded;
  // if the angle has been exceeded and now the angle is below turn off the light 

  if(rightOn == true) {
    if(angleDiff >= rightThreshold && angleDiff <= 180){
      rightThresholdExceeded = true;
      Serial.println("Right Passed"); 
    }

    if(rightThresholdExceeded == true && angleDiff < (rightThreshold - 1)) {
      rightOn = false; 
      rightThresholdExceeded = false; 
      Serial.println("Right Under"); 
    }
  }


  if(leftOn == true) {
    if(angleDiff <= leftThreshold && angleDiff > 270){
      leftThresholdExceeded = true;
      Serial.println("Left Passed"); 
    }

    if(leftThresholdExceeded == true && angleDiff > (leftThreshold + 1)) {
      leftOn = false; 
      leftThresholdExceeded = false; 
      Serial.println("Left Below"); 
    }
  }






// automatic turn on 
// if the mode for automtic turn has been enabled
  if(mode == 3) {

    if(angleDiff >= rightThreshold && angleDiff <=90 && rightOn == false && leftOn == false) { 
      rightOn = true; 
      rightThresholdExceeded = true;

    }

    if(angleDiff <= leftThreshold && angleDiff >= 270 && leftOn == false && rightOn == false) { 
      leftOn = true; 
      leftThresholdExceeded = true;
    }

  }
  //vTaskDelay(100 / portTICK_PERIOD_MS);

}


