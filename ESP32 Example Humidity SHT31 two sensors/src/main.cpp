/* 
 * Purpose: Read & print from two SHT31 humidity sensors.
 * This code uses both I2C channels on the ESP32.
 * Modified from Rob Tillaart's SHT31 library.
 */

#include "SHT31.h"

#define SHT31_ADDRESS   0x45
// note: second address at 0x44

SHT31 SHT_1(SHT31_ADDRESS, &Wire);  // First sensor on default I2C bus (GPIO 21, 22)
SHT31 SHT_2(SHT31_ADDRESS, &Wire1); // Second sensor on second I2C bus

bool b1, b2;

void scanI2C(TwoWire &wire) {
  int deviceCount = 0;

  for (byte address = 1; address < 127; address++) {
      wire.beginTransmission(address);
      if (wire.endTransmission() == 0) {
          Serial.print("Device found at 0x");
          Serial.println(address, HEX);
          deviceCount++;
      }
  }

  if (deviceCount == 0) Serial.println("No I2C devices found");
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C buses
  Wire.begin(); // Default ESP32 I2C pins 21 (SDA), 22 (SCL)
  Wire1.begin(25, 26, 100000); // SDA = 25, SCL = 26
  b1 = SHT_1.begin();
  b2 = SHT_2.begin();

  // Verify device addresses
  Serial.println("Scanning I2C bus 0...");
  scanI2C(Wire);
  Serial.println("Scanning I2C bus 1...");
  scanI2C(Wire1);

  //  see if they are connected
  Serial.print("BEGIN:\t");
  Serial.print(b1);
  Serial.print("\t");
  Serial.print(b2);
  Serial.println();
}

void loop() {
  //  read all sensors that are found
  if (b1) SHT_1.read();
  if (b2) SHT_2.read();

  Serial.print(SHT_1.getTemperature(), 1);
  Serial.print("\t");
  Serial.print(SHT_2.getTemperature(), 1);
  Serial.print("\t");
  Serial.print(SHT_1.getHumidity(), 1);
  Serial.print("\t");
  Serial.print(SHT_2.getHumidity(), 1);
  Serial.println();
  
  delay(1000);
}
