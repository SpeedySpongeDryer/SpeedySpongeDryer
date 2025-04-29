/* 
 * Purpose: Read & print from two DHT20 and two SHT31 humidity sensors.
 * This code uses both I2C channels on the ESP32.
 * Uses a moving average filter and all non-blocking code.
 * Modified from Rob Tillaart's DHT20 and SHT31 libraries.
 */

 #include "DHT20.h"
 #include "SHT31.h"
 
 #define SHT31_ADDRESS   0x45  // Default SHT31 address
 #define READ_INTERVAL   1500  // Sensor reading interval in ms
 #define FILTER_SIZE     10    // Size of moving average filter
 #define PROBE_PIN       15
 #define SIGNAL_PIN      16
 
 DHT20 DHT_1(&Wire);  // First two sensors on default I2C bus (GPIO 21, 22)
 SHT31 SHT_1(SHT31_ADDRESS, &Wire);
 DHT20 DHT_2(&Wire1); // Other sensors on second I2C bus
 SHT31 SHT_2(SHT31_ADDRESS, &Wire1); 
 
 // Global temp/humidity variables for printing in main loop
 float dht1_temp, dht1_hum, dht2_temp, dht2_hum, sht1_temp, sht1_hum, sht2_temp, sht2_hum;
 uint16_t probeReading = 0;
 
 // Timing vairables
  unsigned long lastReadTime = 0;
  float count = 0;
 
 // Moving average variables
  float dht1_hum_buffer[FILTER_SIZE] = {0};
  float dht2_hum_buffer[FILTER_SIZE] = {0};
  float sht1_hum_buffer[FILTER_SIZE] = {0};
  float sht2_hum_buffer[FILTER_SIZE] = {0};
  int bufferIndex = 0;
  bool bufferFilled = false;
 
 // Function declarations
  void readDHT20(DHT20 &sensor, const char *type, int sensorID);
  void readSHT31(SHT31 &sensor, const char *type, int sensorID);
  float getMovingAverage(float buffer[]);
 
 void setup() {
   Serial.begin(115200);
 
   pinMode(PROBE_PIN, INPUT);
   pinMode(SIGNAL_PIN, OUTPUT);
   
   // Initialize the default I2C bus
   Wire.begin(); // Default ESP32 I2C pins 21 (SDA), 22 (SCL)
   DHT_1.begin();
   SHT_1.begin();
 
   // Initialize the second I2C bus with custom pins
   Wire1.begin(25, 26, 100000); // SDA = 25, SCL = 26
   DHT_2.begin();
   SHT_2.begin();
   
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
    //  digitalWrite(SIGNAL_PIN, HIGH);
    //  probeReading = analogRead(PROBE_PIN);
    //  digitalWrite(SIGNAL_PIN, LOW);
 
     // Print to Teleplot
    //  Serial.printf(">Probe:%.1f:%d\n", count, probeReading);
     // Serial.printf(">DHT_temp_in:%.1f:%.2f\n", count, dht2_temp);
     Serial.printf(">SHT_temp_in:%.1f:%.2f\n", count, sht2_temp);
     // Serial.printf(">DHT_temp_out:%.1f:%.2f\n", count, dht1_temp);
     // Serial.printf(">SHT_temp_out:%.1f:%.2f\n", count, sht1_temp);
     Serial.printf(">DHT_humid_in:%.1f:%.2f\n", count, dht2_hum);
     Serial.printf(">SHT_humid_in:%.1f:%.2f\n", count, sht2_hum);
     Serial.printf(">DHT_humid_out:%.1f:%.2f\n", count, dht1_hum);
     Serial.printf(">SHT_humid_out:%.1f:%.2f\n", count, sht1_hum);
     count += 1.5;
     
     // Increment buffer index for moving average
     bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
     if (bufferIndex == 0) { bufferFilled = true; }
   }
 }
 
 void readDHT20(DHT20 &sensor, const char *type, int sensorID) {
   uint32_t start = micros();
   int status = sensor.read();
   uint32_t stop = micros();
 
   if (status == DHT20_OK) {
     float filteredHumidity, humidity = sensor.getHumidity();
     float temperature = sensor.getTemperature();
     
     // Update moving average buffer and calculate filtered value
     if (sensorID == 1) {
       dht1_hum_buffer[bufferIndex] = humidity;
       filteredHumidity = getMovingAverage(dht1_hum_buffer);
       dht1_hum = filteredHumidity;
       dht1_temp = temperature;
     } else {
       dht2_hum_buffer[bufferIndex] = humidity;
       filteredHumidity = getMovingAverage(dht2_hum_buffer);
       dht2_hum = filteredHumidity;
       dht2_temp = temperature;
     }
   }
 }
 
 void readSHT31(SHT31 &sensor, const char *type, int sensorID) {
   uint32_t start = micros();
   bool status = sensor.read();
   uint32_t stop = micros();
 
   if (status) {
     float filteredHumidity, humidity = sensor.getHumidity();
     float temperature = sensor.getTemperature();
     
     // Update moving average buffer and calculate filtered value
     if (sensorID == 1) {
       sht1_hum_buffer[bufferIndex] = humidity;
       filteredHumidity = getMovingAverage(sht1_hum_buffer);
       sht1_hum = filteredHumidity;
       sht1_temp = temperature;
     } else {
       sht2_hum_buffer[bufferIndex] = humidity;
       filteredHumidity = getMovingAverage(sht2_hum_buffer);
       sht2_hum = filteredHumidity;
       sht2_temp = temperature;
     }
   }
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
 