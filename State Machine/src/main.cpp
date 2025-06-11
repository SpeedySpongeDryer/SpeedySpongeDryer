/* 
 * Purpose: State Machine v1.0 for the Speedy Sponge Dryer
 */

#include <Arduino.h>
#include <driver/rtc_io.h>
#include "DHT20.h"
#include "MotorControl.h"

// Pin definitions
#define WAKEUP_PIN (gpio_num_t)13 // For configuring wakeup source
#define IR_D0  13     // TCRT5000 digital output (active low)
#define IR_A0  12     // TCRT5000 analog output

#define DHT_VCC 27    // PIN27 set to HIGH in setup()
#define DHT_SDA 26    // DHT20 I2C data line
#define DHT_GND 25    // PIN25 set to LOW in setup()
#define DHT_SCL 33    // DHT20 I2C clock line

// Motor control pins - PWM and direction pairs
#define MOTOR1_PWM 18
#define MOTOR1_DIR 19
#define MOTOR2_PWM 21
#define MOTOR2_DIR 22
#define MOTORS_ADC 4

// Fan control
#define FAN_PIN  (gpio_num_t)23

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
DHT20 DHT(&Wire1);
RTC_DATA_ATTR SpongeState state = SLEEP;
MotorControl motors(MOTOR1_PWM, MOTOR1_DIR, MOTORS_ADC, MOTOR2_PWM, MOTOR2_DIR);
unsigned long squeezeTime = 0, squeezeStartTime = 0, dryTime = 0, dryStartTime = 0, ejectStartTime = 0;
float temperature = 0, humidity = 0;

// Function declarations
void transitionTo(SpongeState newState);
void handleStandbyState();
void handleSleepState();
void handleSqueezeState();
void handleDryState();
void handleEjectState();
void calculateDryTime();
void printStateTransition(SpongeState newState);
void printMotorsVoltage(int msPrintInterval);

void setup()
{
  Serial.begin(115200);

  // TCRT5000 setup
  pinMode(IR_D0, INPUT);
  pinMode(IR_A0, INPUT);

  // DHT20 setup
  pinMode(DHT_VCC, OUTPUT);
  pinMode(DHT_GND, OUTPUT);
  digitalWrite(DHT_VCC, HIGH);
  digitalWrite(DHT_GND, LOW);
  Wire1.begin(DHT_SDA, DHT_SCL, 100000);
  DHT.begin();
  
  // Motors setup
  motors.begin();
  motors.setupADCInterrupt();

  // Fan setup
  rtc_gpio_hold_dis(FAN_PIN);        // Release held pin from previous deep sleep
  pinMode(FAN_PIN, OUTPUT);          // Set as normal GPIO output
  digitalWrite(FAN_PIN, LOW);        // Force the pin LOW

  Serial.println("Speedy Sponge Dryer State Machine started \n");
}


void loop()
{
  // State machine implementation
  switch (state) {
    case SLEEP:   handleSleepState();   break;
    case SQUEEZE: handleSqueezeState(); break;
    case DRY:     handleDryState();     break;
    case STANDBY: handleStandbyState(); break;
    case EJECT:   handleEjectState();   break;
  }  
}

void transitionTo(SpongeState newState)
{
  printStateTransition(newState);

  // One exit action:
  // If coming from DRY, turn the fan off
  if(state == DRY) {digitalWrite(FAN_PIN, LOW);}

  state = newState; // Update the state BEFORE going to sleep
  
  // Handle entry actions for new state
  switch (newState) {
    case SLEEP:
      // Entry actions for Sleep state
      motors.stopAllMotors();

      // Release I2C bus before sleep
      Wire1.end();

      // Use RTC IO to keep fan off during sleep
      rtc_gpio_set_direction(FAN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_set_level(FAN_PIN, 0);  // Force LOW
      rtc_gpio_hold_en(FAN_PIN);       // Hold this state during sleep
      
      // Configure wake-up source and enter Deep Sleep
      esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, LOW);  // Wake up when TCRT5000 detects object
      Serial.println("going to deep sleep");
      esp_deep_sleep_start();
      break;
      
    case SQUEEZE:
      // Entry actions for Squeeze state
      squeezeStartTime = millis();
      
      // Start both motors
      Serial.println("Starting both motors forward");
      motors.startMotorsForward();

      // Take DHT20 reading (humidity and temperature)
      DHT.read();
      humidity = DHT.getHumidity();
      temperature = DHT.getTemperature();
      Serial.print("Humidity: "); Serial.print(humidity); 
      Serial.print("%, Temperature: "); Serial.print(temperature); Serial.println("°C");
      break;

    case DRY:
      // Entry actions for Dry state
      // Stop motors and activate fan
      motors.stopAllMotors();
      digitalWrite(FAN_PIN, HIGH);
      
      // Save timestamp for dry timing and calculate dry time
      dryStartTime = millis();
      calculateDryTime();
      break;

    case STANDBY:
      // Entry actions for Standby state

      // Use RTC IO to keep fan off during sleep
      rtc_gpio_set_direction(FAN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_set_level(FAN_PIN, 0);  // Force LOW
      rtc_gpio_hold_en(FAN_PIN);       // Hold this state during sleep

      // Configure wake-up source and enter Deep Sleep
      esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, LOW);  // Wake up when PIN13 goes LOW
      Serial.println("going to deep sleep");
      esp_deep_sleep_start();
      break;

    case EJECT:
      // Entry actions for Eject state
      // Save timestamp for debouncing period
      ejectStartTime = millis();

      // Start motors in backward direction for ejection
      motors.startMotorsBackward();
      break;
  }
}

void handleSleepState()
{
  // This should only execute after waking from deep sleep
  // PIN13 interrupt woke us up = TCRT5000 detected an object (hopefully a sponge)
  if (digitalRead(IR_D0) == LOW) {transitionTo(SQUEEZE);}
}

void handleSqueezeState()
{
  // Check if motor should stop due to voltage exceeding threshold
  // The MotorControl class handles the ADC monitoring through interrupts
  // if (MotorControl::shouldStop) {
  //   Serial.println("Stopping motor due to high voltage, aborting squeeze");
  //   motor1.stopMotor();
  //   motor2.stopMotor();
  //   MotorControl::shouldStop = false;  // Reset the flag
  //   transitionTo(EJECT);
  //   return;
  // }
  // printMotorsVoltage(100); // Print ADC readings 10 times per second
  
  // Two-stage squeeze process:
  // 1. First wait until the sensor no longer detects the sponge (IR_D0 goes HIGH)
  // 2. Then wait for the sponge to fully enter the evaporation chamber (timer based)
  
  static bool spongeDetected = true;
  static unsigned long startTime = 0;
  static uint8_t numSqueezes = 0;
  
  if (spongeDetected) {
    // Stage 1: Wait for the sponge to clear the sensor
    if (digitalRead(IR_D0) == HIGH) {
      Serial.println("Sponge passed IR sensor");
      spongeDetected = false;
      startTime = millis();
    }
  } else {
    // Stage 2: Wait for the sponge to fully enter the evap chamber
    if (millis() - startTime >= 1500) {
      transitionTo(DRY);
    }
  }
}

void handleDryState()
{
  // Check if drying time is up
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - dryStartTime;
  
  // Eject sponge if user activates the TCRT5000
  if (digitalRead(IR_D0) == LOW) {transitionTo(EJECT);}
  // Otherwise check if the dryTime is up. (dryTime depends on ambient 
  // temperature & humidity; value set in calculateDryTime())
  else if (elapsedTime >= dryTime) {
    Serial.println("Drying time completed");
    transitionTo(STANDBY);
  }
}

void handleStandbyState()
{
  // This should only execute after waking from deep sleep;
  // triggered by the user activating the TCRT5000 (IR_D0 is LOW)
  if (digitalRead(IR_D0) == LOW) {transitionTo(EJECT);}
}

void handleEjectState()
{
  // printMotorsVoltage(100); // Print ADC readings 10 times per second

  // Two-stage ejection process:
  // 1. First wait for the debouncing period to pass
  // 2. Then wait until the sponge passes and sensor no longer detects it (IR_D0 goes HIGH)
  
  static bool debouncing = true;
  
  if (debouncing) {
    // Stage 1: Observe debouncing period of 4s
    if (millis() - ejectStartTime >= 4000) {
      Serial.println("Debouncing period complete");
      debouncing = false;
    }
  } else {
    // Stage 2: Wait for the sponge to fully pass the sensor
    if (digitalRead(IR_D0) == HIGH) {
      Serial.println("Sponge fully ejected");
      debouncing = true;  // Reset for next time
      transitionTo(SLEEP);
    }
  }
}

void calculateDryTime()
{
  // FORMULA TBD. FOR NOW:
  dryTime = DRY_TIMEUP;
}

void printStateTransition(SpongeState newState)
{
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
}

void printMotorsVoltage(int msPrintInterval)
{
  static unsigned long lastPrintTime = 0;
  unsigned long now = millis();
  if (now - lastPrintTime >= msPrintInterval) {
    lastPrintTime = now;
    int adcValue = analogRead(MOTORS_ADC);      // Read ADC value
    float voltage = adcValue * (3.3 / 4095.0);  // Convert to voltage
    Serial.print("Voltage: ");  // Print voltage value
    Serial.println(voltage, 3); // Print with 3 decimal places
  }
}
