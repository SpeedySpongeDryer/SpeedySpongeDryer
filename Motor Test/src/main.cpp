/* 
 * Purpose: test motor functionality of Speedy Sponge Dryer State Machine v1.0
 * This program contains the pieces of the state machine that have motor actions.
 */

#include <Arduino.h>
#include "MotorControl.h"

#define IR_D0  13     // TCRT5000 digital output (active low)
#define MOTOR1_PWM1 5
#define MOTOR1_PWM2 18
#define MOTOR2_PWM1 19
#define MOTOR2_PWM2 21
#define MOTOR1_ADC 0
#define MOTOR2_ADC 0

// States
typedef enum {
    SLEEP,
    SQUEEZE,
    DRY,
    EJECT,
    STANDBY  
  } SpongeState;
SpongeState state = SLEEP;

MotorControl motor1(MOTOR1_PWM1, MOTOR1_PWM2, MOTOR1_ADC);
MotorControl motor2(MOTOR2_PWM1, MOTOR2_PWM2, MOTOR2_ADC);
unsigned long dryStartTime = 0;

// Function declarations
void transitionTo(SpongeState newState);
void handleStandbyState();
void handleSleepState();
void handleSqueezeState();
void handleDryState();
void handleEjectState();

void setup() {
    Serial.begin(115200);
    motor1.begin();
    motor1.setupADCInterrupt();
    motor2.begin();
    motor2.setupADCInterrupt();

    // TCRT5000 setup for motor activation
    pinMode(IR_D0, INPUT);
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
  state = newState; // Update the state  
  // Handle entry actions for new state
  switch (newState) {
    case SLEEP:
      motor1.stopMotor();
      motor2.stopMotor();
      break;
      
    case SQUEEZE:
      Serial.println("Entering SQUEEZE state");
      motor1.startMotorForward();
      motor2.startMotorForward();
      break;

    case DRY:
      motor1.stopMotor();
      motor2.stopMotor();
      // Save timestamp for dry timing
      dryStartTime = millis();
      break;

    case STANDBY:
      break;

    case EJECT:
      motor1.startMotorBackward();
      motor2.startMotorBackward();
      break;
  }
}

void handleSleepState()
{
  if (digitalRead(IR_D0) == LOW) {transitionTo(SQUEEZE);}
}

void handleSqueezeState()
{
  // Check if motor should stop due to voltage exceeding threshold
  // The MotorControl class handles the ADC monitoring through interrupts
  if (MotorControl::shouldStop) {
    Serial.println("Stopping motor due to high voltage, aborting squeeze");
    motor1.stopMotor();
    motor2.stopMotor();
    MotorControl::shouldStop = false;  // Reset the flag
    transitionTo(EJECT);
    return;
  }

  // Check if the motor has finished squeezing (detected by IR_D0 going HIGH)
  if (digitalRead(IR_D0) == HIGH) {
    Serial.println("Squeeze completed");
    transitionTo(DRY);
  } 
}

void handleDryState()
{
  // Check if drying time is up
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - dryStartTime;
  
  // Eject sponge if user activates the TCRT5000
  if (digitalRead(IR_D0) == LOW) {transitionTo(EJECT);}
  // Otherwise wait for 5s to pass
  else if (elapsedTime >= 5000) {
    Serial.println("Drying time completed");
    transitionTo(EJECT);
  }
}

void handleStandbyState()
{
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