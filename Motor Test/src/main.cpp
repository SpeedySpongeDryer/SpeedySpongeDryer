/* 
 * Purpose: test motor functionality of Speedy Sponge Dryer State Machine v1.0
 * This program contains the pieces of the state machine that have motor actions.
 */

#include <Arduino.h>
// #include "MotorControl.h"

#define IR_D0  13     // TCRT5000 digital output (active low)
#define MOTOR1_PWM1 5
#define MOTOR1_PWM2 18
#define MOTOR2_PWM1 19
#define MOTOR2_PWM2 21
// #define MOTOR1_ADC 2
// #define MOTOR2_ADC 4

// States
typedef enum {
    SLEEP,
    SQUEEZE,
    DRY,
    EJECT,
    STANDBY  
  } SpongeState;
SpongeState state = SLEEP;

// MotorControl motor1(MOTOR1_PWM1, MOTOR1_PWM2, MOTOR1_ADC);
// MotorControl motor2(MOTOR2_PWM1, MOTOR2_PWM2, MOTOR2_ADC);
unsigned long dryStartTime = 0;

// Function declarations
void transitionTo(SpongeState newState);
void handleStandbyState();
void handleSleepState();
void handleSqueezeState();
void handleDryState();
void handleEjectState();
void startMotorForward();
void startMotorBackward();
void stopMotor();

void setup() {
    Serial.begin(115200);
    // motor1.begin();
    // motor1.setupADCInterrupt();
    // motor2.begin();
    // motor2.setupADCInterrupt();

    // Set up motor pins as outputs
    pinMode(MOTOR1_PWM1, OUTPUT);
    pinMode(MOTOR1_PWM2, OUTPUT);
    pinMode(MOTOR2_PWM1, OUTPUT);
    pinMode(MOTOR2_PWM2, OUTPUT);
    
    // Stop motors initially
    stopMotor();

    // TCRT5000 setup for motor activation
    pinMode(IR_D0, INPUT);

    Serial.println("Simplified Sponge Dryer State Machine started");
}

void loop() {
  // State machine implementation
  switch (state) {
    case STANDBY: handleStandbyState(); break;
    case SLEEP:   handleSleepState();   break;
    case SQUEEZE: handleSqueezeState(); break;
    case DRY:     handleDryState();     break;
    case EJECT:   handleEjectState();   break;
  }  
}

void transitionTo(SpongeState newState) {
    Serial.print("\nTransitioning from ");
    switch (state) {
        case SLEEP:   Serial.print("SLEEP");   break;
        case SQUEEZE: Serial.print("SQUEEZE"); break;
        case DRY:     Serial.print("DRY");     break;
        case EJECT:   Serial.print("EJECT");   break;
        case STANDBY: Serial.print("STANDBY"); break;
    }
    
    Serial.print(" to ");
    switch (newState) {
        case SLEEP:   Serial.println("SLEEP");   break;
        case SQUEEZE: Serial.println("SQUEEZE"); break;
        case DRY:     Serial.println("DRY");     break;
        case EJECT:   Serial.println("EJECT");   break;
        case STANDBY: Serial.println("STANDBY"); break;
    }

  state = newState; // Update the state 

  // Handle entry actions for new state
  switch (newState) {
    case SLEEP:    stopMotor();           break;      
    case SQUEEZE:  startMotorForward();   break;
    case DRY:      stopMotor();
                   dryStartTime = millis(); // Save timestamp for dry timing
                                          break;
    case STANDBY:  delay(1000);           break;
    case EJECT:    startMotorBackward();  break;
  }
}

void handleSleepState()
{
  if (digitalRead(IR_D0) == LOW) {transitionTo(SQUEEZE);}
}

void handleSqueezeState()
{
  // Check if the motor has finished squeezing (detected by IR_D0 going HIGH)
  if (digitalRead(IR_D0) == HIGH) {transitionTo(DRY);} 
}

void handleDryState()
{
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - dryStartTime;
  
  // Eject sponge if user activates the TCRT5000
  if (digitalRead(IR_D0) == LOW) {transitionTo(EJECT);}
  // Otherwise wait for 10s to pass
  else if (elapsedTime >= 10000) {
    Serial.println("Drying time completed");
    transitionTo(EJECT);
  }
}

void handleStandbyState()
{
  // Eject sponge if user activates the TCRT5000
  if (digitalRead(IR_D0) == LOW) {transitionTo(EJECT);}
}

void handleEjectState()
{
  // Two-stage ejection process:
  // 1. First wait until the sensor detects the sponge (IR_D0 goes LOW)
  // 2. Then wait until the sponge passes and sensor no longer detects it (IR_D0 goes HIGH)
  
  static bool spongeDetected = false;
  
  if (!spongeDetected) {
    // Stage 1: Wait for the sponge to reach the sensor
    if (digitalRead(IR_D0) == LOW) {
      Serial.println("Sponge detected during ejection");
      spongeDetected = true;
    }
  } else {
    // Stage 2: Wait for the sponge to fully pass the sensor
    if (digitalRead(IR_D0) == HIGH) {
      Serial.println("Sponge fully ejected");
      spongeDetected = false;  // Reset for next time
      transitionTo(SLEEP);
    }
  }
}

// Simple motor control functions
void startMotorForward() {
    digitalWrite(MOTOR1_PWM1, HIGH);
    digitalWrite(MOTOR1_PWM2, LOW);
    digitalWrite(MOTOR2_PWM1, HIGH);
    digitalWrite(MOTOR2_PWM2, LOW);
    Serial.println("Motors rotating forward");
}

void startMotorBackward() {
    digitalWrite(MOTOR1_PWM1, LOW);
    digitalWrite(MOTOR1_PWM2, HIGH);
    digitalWrite(MOTOR2_PWM1, LOW);
    digitalWrite(MOTOR2_PWM2, HIGH);
    Serial.println("Motors rotating backward");
}

void stopMotor() {
    digitalWrite(MOTOR1_PWM1, LOW);
    digitalWrite(MOTOR1_PWM2, LOW);
    digitalWrite(MOTOR2_PWM1, LOW);
    digitalWrite(MOTOR2_PWM2, LOW);
    Serial.println("Motors stopped");
}