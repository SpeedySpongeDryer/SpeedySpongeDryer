/* 
 * Purpose: Read & print from two DHT20 and two SHT31 humidity sensors.
 * This code uses both I2C channels on the ESP32.
 * Uses a moving average filter and all non-blocking code.
 * Modified from Rob Tillaart's DHT20 and SHT31 libraries.
 */

#include "DHT20.h"
#include "SHT31.h"

#define SHT31_ADDRESS   0x45  // Default SHT31 address
#define READ_INTERVAL   1500  // Read interval in ms
#define FILTER_SIZE     5     // Size of moving average filter

DHT20 DHT_1(&Wire);  // First two sensors on default I2C bus (GPIO 21, 22)
SHT31 SHT_1(SHT31_ADDRESS, &Wire);
DHT20 DHT_2(&Wire1); // Other sensors on second I2C bus
SHT31 SHT_2(SHT31_ADDRESS, &Wire1); 

// Timing vairable
 unsigned long lastReadTime = 0;

// Moving average variables
 float dht1_hum_buffer[FILTER_SIZE] = {0};
 float dht2_hum_buffer[FILTER_SIZE] = {0};
 float sht1_hum_buffer[FILTER_SIZE] = {0};
 float sht2_hum_buffer[FILTER_SIZE] = {0};
 int bufferIndex = 0;
 bool bufferFilled = false;

// Function declarations
 void scanI2C(TwoWire &wire, const char* busName);
 void readDHT20(DHT20 &sensor, const char *type, int sensorID);
 void readSHT31(SHT31 &sensor, const char *type, int sensorID);
 float getMovingAverage(float buffer[]);

void setup() {
  Serial.begin(115200);

  // Initialize the default I2C bus
  Wire.begin(); // Default ESP32 I2C pins 21 (SDA), 22 (SCL)
  DHT_1.begin();
  SHT_1.begin();

  // Initialize the second I2C bus with custom pins
  Wire1.begin(25, 26, 100000); // SDA = 25, SCL = 26
  DHT_2.begin();
  SHT_2.begin();

  // Verify device addresses
  Serial.println("Scanning I2C bus 0 (GPIO 21, 22)...");
  scanI2C(Wire, "Bus 0");
  Serial.println("Scanning I2C bus 1 (GPIO 25, 26)...");
  scanI2C(Wire1, "Bus 1");

  Serial.println();
  Serial.println("Type\tSensor\tHumidity (%)\tTemp (°C)\tTime (µs)\tStatus");
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Non-blocking sensor reads
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Read the sensors
    readDHT20(DHT_1, "DHT20", 1);
    readDHT20(DHT_2, "DHT20", 2);
    readSHT31(SHT_1, "SHT31", 1);
    readSHT31(SHT_2, "SHT31", 2);
    Serial.println("______________________________________________________\n");
    
    // Increment buffer index for moving average
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    if (bufferIndex == 0) { bufferFilled = true; }
  }
}

void scanI2C(TwoWire &wire, const char* busName) {
  int deviceCount = 0;

  for (byte address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    if (wire.endTransmission() == 0) {
      Serial.print(busName);
      Serial.print(" - Device found at 0x");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.print(busName);
    Serial.println(" - No I2C devices found");
  }
}

void readDHT20(DHT20 &sensor, const char *type, int sensorID) {
  uint32_t start = micros();
  int status = sensor.read();
  uint32_t stop = micros();

  Serial.print(type);     Serial.print("\t");
  Serial.print(sensorID); Serial.print("\t");

  if (status == DHT20_OK) {
    float filteredHumidity, humidity = sensor.getHumidity();
    float temperature = sensor.getTemperature();
    
    // Update moving average buffer and calculate filtered value
    if (sensorID == 1) {
      dht1_hum_buffer[bufferIndex] = humidity;
      filteredHumidity = getMovingAverage(dht1_hum_buffer);
    } else {
      dht2_hum_buffer[bufferIndex] = humidity;
      filteredHumidity = getMovingAverage(dht2_hum_buffer);
    }
    
    // Print the values
    Serial.print(filteredHumidity, 1);    Serial.print("\t\t");
    Serial.print(temperature, 1);         Serial.print("\t\t");
    Serial.print(stop - start);           Serial.print("\t\tOK");
  } else {
    Serial.print("ERROR: ");
    switch (status) {
      case DHT20_ERROR_CHECKSUM:        Serial.print("Checksum error");       break;
      case DHT20_ERROR_CONNECT:         Serial.print("Connect error");        break;
      case DHT20_MISSING_BYTES:         Serial.print("Missing bytes");        break;
      case DHT20_ERROR_BYTES_ALL_ZERO:  Serial.print("All bytes read zero");  break;
      case DHT20_ERROR_READ_TIMEOUT:    Serial.print("Read timeout");         break;
      case DHT20_ERROR_LASTREAD:        Serial.print("Read too fast");        break;
      default:                          Serial.print("Unknown error");        break;
    }
  }
  Serial.println();
}

void readSHT31(SHT31 &sensor, const char *type, int sensorID) {
  uint32_t start = micros();
  bool status = sensor.read();
  uint32_t stop = micros();

  Serial.print(type);     Serial.print("\t");
  Serial.print(sensorID); Serial.print("\t");

  if (status) {
    float filteredHumidity, humidity = sensor.getHumidity();
    float temperature = sensor.getTemperature();
    
    // Update moving average buffer and calculate filtered value
    if (sensorID == 1) {
      sht1_hum_buffer[bufferIndex] = humidity;
      filteredHumidity = getMovingAverage(sht1_hum_buffer);
    } else {
      sht2_hum_buffer[bufferIndex] = humidity;
      filteredHumidity = getMovingAverage(sht2_hum_buffer);
    }
    
    // Print the values
    Serial.print(filteredHumidity, 1);    Serial.print("\t\t");
    Serial.print(temperature, 1);         Serial.print("\t\t");
    Serial.print(stop - start);           Serial.print("\t\tOK");
  } else {
    Serial.print("ERROR: ");
    Serial.print("Communication error");
  }
  Serial.println();
}

float getMovingAverage(float buffer[]) {
  if (!bufferFilled) {
    // If buffer not filled yet, just return the current value
    float sum = 0;
    for (int i = 0; i <= bufferIndex; i++) {
      sum += buffer[i];
    }
    return sum / (bufferIndex + 1);
  }
  
  // If buffer is filled, calculate average of all values
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SIZE;
}
