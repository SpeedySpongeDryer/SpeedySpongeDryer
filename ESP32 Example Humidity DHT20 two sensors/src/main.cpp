//
//  Purpose: Read & print from two DHT20 humidity sensors.
//  The DHT20 has a single address so both I2C buses must be used.
//  Modified from Rob Tillaart's DHT20 library
//
//          +--------------+
//  VDD ----| 1            |
//  SDA ----| 2    DHT20   |
//  GND ----| 3            |
//  SCL ----| 4            |
//          +--------------+


#include "DHT20.h"

void readSensor(DHT20 &sensor, const char *type, int sensorID);

DHT20 DHT1(&Wire);  // First sensor on default I2C bus (GPIO 21, 22)
DHT20 DHT2(&Wire1); // Second sensor on second I2C bus

uint32_t count = 0;
float h1 = 0, h2 = 0;


void scanI2C(TwoWire *wire) {
  for (byte address = 1; address < 127; address++) {
      wire->beginTransmission(address);
      if (wire->endTransmission() == 0) {
          Serial.print("Device found at 0x");
          Serial.println(address, HEX);
      }
  }
}



void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("DHT20 LIBRARY VERSION: ");
  Serial.println(DHT20_LIB_VERSION);
  Serial.println();

  // Initialize the first I2C bus
  Wire.begin(); // Default ESP32 I2C pins 21 (SDA), 22 (SCL)
  DHT1.begin();

  // Initialize the second I2C bus with custom pins
  Wire1.begin(25, 26, 100000); // SDA = 25, SCL = 26
  DHT2.begin();

  // Verify device addresses
  scanI2C(&Wire);
  scanI2C(&Wire1);

  delay(2000);
}


void loop()
{
  if (millis() - DHT2.lastRead() >= 1000)
  {
    // h1 = readSensor(DHT1, "DHT20", 1);
    // h2 = DHT2.getHumidity();

    // Serial.printf("%f\t%f\n", h1, h2);

    // print to Teleplot
    // Serial.printf(">h1:%d:%.1f\n", count, h1);
    // Serial.printf(">h2:%d:%.1f\n", count, h2);
    // count++;

    //  READ DATA

    Serial.println();
    Serial.println("Type\tSensor\tHumidity (%)\tTemp (°C)\tTime (µs)\tStatus");

    readSensor(DHT1, "DHT20", 1);
    readSensor(DHT2, "DHT20", 2);
  }
}


void readSensor(DHT20 &sensor, const char *type, int sensorID)
{
  uint32_t start = micros();
  int status = sensor.read();
  uint32_t stop = micros();


  Serial.print(type);
  Serial.print(" \t");
  Serial.print(sensorID);
  Serial.print("\t");

  if (status == DHT20_OK)
  {
    Serial.print(sensor.getHumidity(), 1);
    Serial.print("\t\t");
    Serial.print(sensor.getTemperature(), 1);
    Serial.print("\t\t");
    Serial.print(stop - start);
    Serial.print("\t\tOK");
  }
  else
  {
    Serial.print("ERROR: ");
    switch (status)
    {
      case DHT20_ERROR_CHECKSUM:
        Serial.print("Checksum error");
        break;
      case DHT20_ERROR_CONNECT:
        Serial.print("Connect error");
        break;
      case DHT20_MISSING_BYTES:
        Serial.print("Missing bytes");
        break;
      case DHT20_ERROR_BYTES_ALL_ZERO:
        Serial.print("All bytes read zero");
        break;
      case DHT20_ERROR_READ_TIMEOUT:
        Serial.print("Read timeout");
        break;
      case DHT20_ERROR_LASTREAD:
        Serial.print("Read too fast");
        break;
      default:
        Serial.print("Unknown error");
        break;
    }
  }
  Serial.println();
}


//  -- END OF FILE --
