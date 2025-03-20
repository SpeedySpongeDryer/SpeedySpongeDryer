// This code will turn on the on-board blue LED
// When the Boot button (pin 0) is pushed the device will enter deep sleep and the LED will turn off


#include <Arduino.h>

#define BOOT_BUTTON 0   // BOOT button (GPIO0)
#define WAKEUP_PIN (gpio_num_t)13  
#define LED_PIN 2     	// Onboard LED (most ESP32 boards)
#define PIN13 13
#define PIN14 14

void setup() {
	pinMode(BOOT_BUTTON, INPUT_PULLUP); // enable BOOT button (internal pull-up)
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);        // turn on board blue LED ON, indicating awake

	Serial.begin(115200);
	Serial.println("ESP32 is awake!");

  pinMode(PIN13, INPUT);
  pinMode(PIN14, INPUT);
}


void loop() {
	if (digitalRead(BOOT_BUTTON) == LOW) {                         	// BOOT button press causes GPIO0 to go LOW

    	Serial.println("BOOT button pressed, going to sleep...");

    	delay(500);                                                	// debounce and allow serial print to complete

    	esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, LOW);             	// enable wake-up when GPIO13 goes LOW
   	 
    	esp_deep_sleep_start();                                    	// enter deep sleep
	}

  Serial.print("Pin 13 = ");
  Serial.print(digitalRead(PIN13));
  Serial.print("\tPin 14 ADC Reading = ");
  Serial.println(analogRead(PIN14));
  delay(500);
}


////////////////////////////////////////////////////////////////////////
// #include <Arduino.h>


// const uint8_t pin13 = 13, pin14 = 14;


// void setup() {
  // Serial.begin(115200); 
  // pinMode(pin13, INPUT);
  // pinMode(pin14, INPUT);
// }


// void loop() {
  // Serial.print("Pin 13 = ");
  // Serial.print(digitalRead(pin13));
  // Serial.print("\tPin 14 ADC Reading = ");
  // Serial.println(analogRead(pin14));
  // delay(500);
// }

////////////////////////////////////////////////////////////////////////
// #include <Arduino.h>


// const uint8_t pin13 = 13, pin14 = 14;
// static volatile bool pin13_HIGH = false, pin13_FALLING = false;


// void IRAM_ATTR PIN13_ISR() {
//   if(digitalRead(pin13)) pin13_HIGH = true;
//   else pin13_FALLING = true;
// }


// void setup() {
//   Serial.begin(115200);
//   pinMode(pin13, INPUT);
//   attachInterrupt(pin13, PIN13_ISR, CHANGE);
//   pinMode(pin14, INPUT);
// }


// void loop() {
//   static unsigned long last_time = 0;

//   if(pin13_HIGH){
//     pin13_HIGH = false; // clear interrupt flag
//     // Debounce
//     unsigned long current_time = millis();
//     if(current_time - last_time > 50) {
//       last_time = current_time;
//       Serial.print("Pin 13 Reading = ");
//       Serial.print(digitalRead(pin13));
//       Serial.print("\tPin 14 ADC Reading = ");
//       Serial.println(analogRead(pin14));
//     }  
//   }
//   else if(pin13_FALLING){
//     pin13_FALLING = false;
//     Serial.println("\nno object detected;");
//     Serial.print("Pin 13 Reading = ");
//     Serial.print(digitalRead(pin13));
//     Serial.print("\tPin 14 ADC Reading = ");
//     Serial.println(analogRead(pin14));
//   }
// }
