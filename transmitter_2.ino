#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

// Receiver ESP32 MAC address
uint8_t receiverMacAddress[] = {0xD4, 0x8A, 0xFC, 0xAA, 0x17, 0x94};

// Flex sensor pins
#define FLEX1_PIN 34
#define FLEX2_PIN 35

// Tilt dead zone to prevent false positives
#define TILT_THRESHOLD 20  
#define FLEX_THRESHOLD 2000  

Adafruit_MPU6050 mpu;

// Structure to hold gesture data
typedef struct {
  int8_t xTilt;  
  int8_t yTilt;  
  bool flex1;    
  bool flex2;    
} GestureData;

GestureData gesture;

// Callback for ESP-NOW send status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Initialized");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(FLEX1_PIN, INPUT);
  pinMode(FLEX2_PIN, INPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  sensors_event_t accel, gyro, temp;
  
  if (!mpu.getEvent(&accel, &gyro, &temp)) {
    Serial.println("MPU6050 Read Error");
    return;
  }

  // Calculate tilt angles
  float tiltX = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;

  // Map tilt angles to -100 to 100 range
  gesture.xTilt = constrain(map(tiltX, -90, 90, -100, 100), -100, 100);
  
  // Read flex sensor values
  int flex1Value = analogRead(FLEX1_PIN);
  int flex2Value = analogRead(FLEX2_PIN);

  // Default: No motion
  int8_t command = 0;

  // Check for **FORWARD** (Flex 1 bent + Tilt RIGHT)
  if (flex1Value > FLEX_THRESHOLD && gesture.xTilt > TILT_THRESHOLD) {
    command = 1;  // Forward
  }
  // Check for **BACKWARD** (Flex 2 bent + Tilt LEFT)
  else if (flex2Value > FLEX_THRESHOLD && gesture.xTilt < -TILT_THRESHOLD) {
    command = -1; // Backward
  }

  // Update gesture data only when valid
  if (command == 1) {
    gesture.xTilt = 50;  // Fixed speed forward
  } else if (command == -1) {
    gesture.xTilt = -50; // Fixed speed backward
  } else {
    gesture.xTilt = 0; // No movement
  }

  // Debug Output
  Serial.print("Flex1: "); Serial.print(flex1Value);
  Serial.print(" | Flex2: "); Serial.print(flex2Value);
  Serial.print(" | X Tilt: "); Serial.print(gesture.xTilt);
  Serial.print(" | Command: "); Serial.println(command == 1 ? "FORWARD" : (command == -1 ? "BACKWARD" : "NONE"));

  // Send data via ESP-NOW
  esp_now_send(receiverMacAddress, (uint8_t *) &gesture, sizeof(gesture));

  delay(100); // Adjust delay as needed
}
