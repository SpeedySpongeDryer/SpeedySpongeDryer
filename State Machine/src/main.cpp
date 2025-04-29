/* 
 * Purpose: State Machine v1.0 for the Speedy Sponge Dryer
 */


#include "DHT20.h"

DHT20 DHT;

// Pin definitions
#define PIN13 13    // TCRT5000 digital output (active low)
#define PIN14 14    // TCRT5000 analog outptu
#define WAKEUP_PIN (gpio_num_t)13
#define MOTOR1_PIN 0        // TBD
#define MOTOR2_PIN 0        // TBD
#define MOTOR1_ADC 0        // TBD
#define MOTOR2_ADC 0        // TBD
#define FAN_PIN   0         // TBD

// Threshold definitions
#define DRY_TIMEUP 1200000  // 20 minutes
#define MOTOR_CURRENT 0     // TBD

// States
typedef enum {
  SLEEP,
  SQUEEZE,
  DRY,
  EJECT,
  STANDBY  
} SpongeState;

// Global variables
SpongeState state = SLEEP;
unsigned long dryStartTime = 0;

// Function declarations
void handleStandbyState();
void handleSleepState();
void handleSqueezeState();
void handleDryState();
void handleEjectState();
void transitionTo(SpongeState newState);


void setup()
{
  Serial.begin(115200);

  pinMode(PIN13, INPUT);
  pinMode(PIN14, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR1_ADC, INPUT);
  pinMode(MOTOR2_ADC, INPUT);

  Wire.begin();
  DHT.begin();    // Default ESP32 I2C pins 21 (SDA), 22 (SCL)

}


void loop()
{
  // State machine implementation
  switch (state) {
    case STANDBY:
      handleStandbyState();
      break;
    case SLEEP:
      handleSleepState();
      break;
    case SQUEEZE:
      handleSqueezeState();
      break;
    case DRY:
      handleDryState();
      break;
    case EJECT:
      handleEjectState();
      break;
  }  
}

void handleStandbyState()
{

}

void handleSleepState()
{

}

void handleSqueezeState()
{

}

void handleDryState()
{

}

void handleEjectState()
{

}

void transitionTo(SpongeState newState)
{

}

