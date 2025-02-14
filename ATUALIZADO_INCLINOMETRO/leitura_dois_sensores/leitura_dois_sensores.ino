#include <PL_ADXL355.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//==============================================================================

// Pin connections for using SPI or I2C interface
// ADXL355 pin    Arduino pin
//                SPI     I2C
// CS/SCL          2      SCL
// MOSI/SDA       MOSI    SDA
// MISO/ASEL      MISO    GND
// SCLK/Vssio     SCLK    GND

PL::ADXL355 adxl355;
//uint8_t spiCsPin = 2;
uint8_t i2cAddress = 0x53;

Adafruit_MPU6050 mpu;
// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//==============================================================================

void setupMPU();

void setup() {

  setupMPU();
  Serial.begin(115200);
  Wire.begin(21, 37);
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  //adxl355.beginSPI(spiCsPin);
  adxl355.beginI2C(i2cAddress);

  // Set ADXL355 range
  adxl355.setRange(range);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  
  // Wait for Serial ready state
  while(!Serial);
}

//==============================================================================

void loop() {
  // Read and print the accelerations
  auto accelerations = adxl355.getAccelerations();
  Serial.print("Accelerations: X: ");
  Serial.print(accelerations.x);
  Serial.print(" g, Y: ");
  Serial.print(accelerations.y);
  Serial.print(" g, Z: ");
  Serial.print(accelerations.z);
  Serial.println(" g");

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);

  delay (1000);
}
