/* 
 * Purpose: test program for fan control in Speedy Sponge Dryer
 */
 
#include <Arduino.h>

// Pin definitions
#define FAN_PWM_PIN  32      // Fan PWM wire tied to this pin
#define FAN_PWM_CHAN 2       // PWM channel (motors use 0 & 1)

// Fan speed definitions
#define FAN_OFF    1         // 0% duty cycle
#define FAN_LOW    64        // 25% duty cycle
#define FAN_MEDIUM 128       // 50% duty cycle
#define FAN_HIGH   192       // 75% duty cycle
#define FAN_MAX    255       // 100% duty cycle

void setFanSpeed(uint8_t speed);


void setup() {
  Serial.begin(115200);
  Serial.println("Fan Hello World - Test Program");
  
  // Fan setup
  ledcSetup(FAN_PWM_CHAN, 25000, 8);         // Channel 2, 25kHz, 8-bit resolution
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHAN);  // Attach pin to channel
  
  // Initially turn fan off
  setFanSpeed(FAN_OFF);
  Serial.println("Fan initialized and set to OFF");
}

void loop() {
  // Test sequence: cycle through different fan speeds
  Serial.println("Setting fan to LOW");
  setFanSpeed(FAN_LOW);
  delay(5000);  // Run for 5 seconds
  
  Serial.println("Setting fan to MEDIUM");
  setFanSpeed(FAN_MEDIUM);
  delay(5000);  // Run for 5 seconds
  
  Serial.println("Setting fan to HIGH");
  setFanSpeed(FAN_HIGH);
  delay(5000);  // Run for 5 seconds
  
  Serial.println("Setting fan to MAX");
  setFanSpeed(FAN_MAX);
  delay(5000);  // Run for 5 seconds
  
  Serial.println("Setting fan to OFF");
  setFanSpeed(FAN_OFF);
  delay(10000);  // Pause for 10 seconds before repeating
}

// Function to set fan speed (0-255)
void setFanSpeed(uint8_t speed) {
  ledcWrite(FAN_PWM_CHAN, speed);
  Serial.print("Fan speed set to: ");
  Serial.print((speed * 100) / 255);
  Serial.println("%");
}
