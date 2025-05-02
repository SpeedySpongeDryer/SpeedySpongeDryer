/* 
 * Purpose: State Machine v1.0 for the Speedy Sponge Dryer
 */


#include "DHT20.h"
#include "MotorControl.h"

// Pin definitions
#define PIN13 13    // TCRT5000 digital output (active low)
#define PIN14 14    // TCRT5000 analog output
#define WAKEUP_PIN (gpio_num_t)13
#define MOTOR1_PWM1 0        // TBD
#define MOTOR1_PWM2 0        // TBD
#define MOTOR2_PWM1 0        // TBD
#define MOTOR2_PWM2 0        // TBD
#define MOTOR1_ADC 0        // TBD
#define MOTOR2_ADC 0        // TBD
#define FAN_PIN   0         // TBD

// Threshold definitions
#define DRY_TIMEUP 1200000  // 20 minutes

// States
typedef enum {
  SLEEP,
  SQUEEZE,
  DRY,
  EJECT,
  STANDBY  
} SpongeState;

// Global variables
DHT20 DHT;
SpongeState state = SLEEP;
MotorControl motor1(MOTOR1_PWM1, MOTOR1_PWM2, MOTOR1_ADC);
MotorControl motor2(MOTOR2_PWM1, MOTOR2_PWM2, MOTOR2_ADC);
unsigned long dryTime = 0, dryStartTime = 0;
float temp = 0, humidity = 0;

// Function declarations
void transitionTo(SpongeState newState);
void handleStandbyState();
void handleSleepState();
void handleSqueezeState();
void handleDryState();
void handleEjectState();
void calculateDryTime();

void setup()
{
  Serial.begin(115200);

  pinMode(PIN13, INPUT);
  pinMode(PIN14, INPUT);
  pinMode(FAN_PIN, OUTPUT);

  Wire.begin();
  DHT.begin();    // Default ESP32 I2C pins 21 (SDA), 22 (SCL)
  
  motor1.begin();
  motor1.setupADCInterrupt();
  motor2.begin();
  motor2.setupADCInterrupt();

}


void loop()
{
  // State machine implementation
  switch (state) {
    case STANDBY: handleStandbyState(); break;
    case SLEEP:   handleSleepState();   break;
    case SQUEEZE: handleSqueezeState(); break;
    case DRY:     handleDryState();     break;
    case EJECT:   handleEjectState();   break;
  }  
}

void transitionTo(SpongeState newState)
{
  // If coming from DRY, turn the fan off
  if(state == DRY) {digitalWrite(FAN_PIN, LOW);}
  
  state = newState; // Update the state
  
  // Handle entry actions for new state
  switch (newState) {
    case SLEEP:
      // Entry actions for Sleep state
      motor1.stopMotor();  // deactivate motors
      motor2.stopMotor();
      
      // Configure wake-up source and enter Deep Sleep
      esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, LOW);  // Wake up when PIN13 goes LOW
      Serial.println("Entering SLEEP state and going to deep sleep");
      esp_deep_sleep_start();
      break;
      
    case SQUEEZE:
      // Entry actions for Squeeze state
      Serial.println("Entering SQUEEZE state");

      // Take DHT20 reading (humidity and temperature)
      float humidity, temperature;
      DHT.read();
      humidity = DHT.getHumidity();
      temperature = DHT.getTemperature();
      Serial.print("Humidity: "); Serial.print(humidity); 
      Serial.print("%, Temperature: "); Serial.print(temperature); Serial.println("°C");

      // Start motors for squeezing
      motor1.startMotorForward();
      motor2.startMotorForward();
      break;

    case DRY:
      // Entry actions for Dry state
      Serial.println("Entering DRY state");
      motor1.stopMotor();  // deactivate motors
      motor2.stopMotor();
      digitalWrite(FAN_PIN, HIGH);  // activate fan
      
      // Save timestamp for dry timing and calculate dry time
      dryStartTime = millis();
      calculateDryTime();
      break;

    case STANDBY:
      // Entry actions for Standby state
      digitalWrite(FAN_PIN, LOW);  // deactivate fan
      
      // Configure wake-up source and enter Deep Sleep
      esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, LOW);  // Wake up when PIN13 goes LOW
      Serial.println("Entering STANDBY state and going to deep sleep");
      esp_deep_sleep_start();
      break;

    case EJECT:
      // Entry actions for Eject state
      Serial.println("Entering EJECT state");

      // Start motors in backward direction for ejection
      motor1.startMotorBackward();
      motor2.startMotorBackward();
      break;
  }
}

void handleSleepState()
{
  // This should only execute after waking from deep sleep
  // PIN13 interrupt woke us up, which means a sponge was inserted
  if (digitalRead(PIN13) == LOW) {transitionTo(SQUEEZE);}
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

  // Check if the motor has finished squeezing (detected by PIN13 going HIGH)
  if (digitalRead(PIN13) == HIGH) {
    Serial.println("Squeeze completed");
    transitionTo(DRY);
  } 
}

void handleDryState()
{
  // Check if drying time is up
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - dryStartTime;
  
  // dryTime depends on ambient temperature & humidity
  // For now, using the default value from the definitions
  if (elapsedTime >= dryTime) {
    Serial.println("Drying time completed");
    transitionTo(EJECT);
  }
}

void handleStandbyState()
{
  // This should only execute after waking from deep sleep;
  // triggered by the user activating the TCRT5000 (PIN13 is LOW)
  if (digitalRead(PIN13) == LOW) {transitionTo(EJECT);}
}

void handleEjectState()
{
  // Two-stage ejection process:
  // 1. First wait until the sensor detects the sponge (PIN13 goes LOW)
  // 2. Then wait until the sponge passes and sensor no longer detects it (PIN13 goes HIGH)
  
  static bool spongeDetected = false;
  
  if (!spongeDetected) {
    // Stage 1: Wait for the sponge to reach the sensor
    if (digitalRead(PIN13) == LOW) {
      Serial.println("Sponge detected during ejection");
      spongeDetected = true;
    }
  } else {
    // Stage 2: Wait for the sponge to fully pass the sensor
    if (digitalRead(PIN13) == HIGH) {
      Serial.println("Sponge fully ejected");
      spongeDetected = false;  // Reset for next time
      transitionTo(SLEEP);
    }
  }
}

void calculateDryTime()
{
  // FORMULA TBD. FOR NOW:
  dryTime = DRY_TIMEUP;
}
