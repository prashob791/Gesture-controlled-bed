#include <WiFi.h>
#include <esp_now.h>

// Motor control pins (MDD10A)
#define PWM1 26  // Motor speed
#define DIR1 27  // Motor direction

// Gesture data structure
typedef struct {
  int8_t xTilt;  
  int8_t yTilt;  
  bool flex1;    
  bool flex2;    
} GestureData;

GestureData gesture;

// Callback function for receiving data
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&gesture, incomingData, sizeof(gesture));

  Serial.print("Flex1: "); Serial.print(gesture.flex1);
  Serial.print(" | Flex2: "); Serial.print(gesture.flex2);
  Serial.print(" | Y Tilt: "); Serial.println(gesture.yTilt);

  controlMotor();
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);

  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

// Motor control function
void controlMotor() {
  int speed;   
  bool dir;    

  if (gesture.flex1 && gesture.yTilt > 20) {  
    speed = map(gesture.yTilt, 20, 100, 100, 255);
    dir = HIGH;  
  } 
  else if (gesture.flex2 && gesture.yTilt < -20) {  
    speed = map(gesture.yTilt, -20, -100, 100, 255);
    dir = LOW;  
  } 
  else {  
    speed = 0;
    dir = HIGH;  
  }

  digitalWrite(DIR1, dir);
  analogWrite(PWM1, speed);
}

// ✅ Add an empty `loop()` function to prevent compilation errors
void loop() {
}
