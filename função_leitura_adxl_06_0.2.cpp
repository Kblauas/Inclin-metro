#include <Arduino.h>
#include <PL_ADXL355.h>
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
uint8_t I2c_SDAPIN=21;
uint8_t I2c_SCLPIN=37;
const float L=0.5;

// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//==============================================================================

void setup() {
  
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  //adxl355.beginSPI(spiCsPin);
  Wire.begin(I2c_SDAPIN, I2c_SCLPIN);
  adxl355.beginI2C(i2cAddress);

  // Set ADXL355 range
  adxl355.setRange(range);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);
  // Wait for Serial ready state
  while(!Serial);
}

//==============================================================================
void lermedidas(){
  int nummedidas = 10;
  float xsoma = 0, ysoma = 0, zsoma = 0;
  float somaOx = 0, somaOy = 0, somaOz = 0;
  float deviacaocumulativax = 0, deviacaocumulativay = 0, deviacaocumulativaz = 0;

  for (int j = 0; j < nummedidas; j++) {
    auto accelerations = adxl355.getAccelerations();
    
    float Ox = atan2(accelerations.x, (sqrt(accelerations.y * accelerations.y)) + (sqrt(accelerations.z * accelerations.z))) * 180 / M_PI;
    float Oy = atan2(accelerations.y, (sqrt(accelerations.x * accelerations.x)) + (sqrt(accelerations.z * accelerations.z))) * 180 / M_PI;
    float Oz = atan2(accelerations.z, (sqrt(accelerations.x * accelerations.x)) + (sqrt(accelerations.y * accelerations.y))) * 180 / M_PI;

    xsoma += accelerations.x;
    ysoma += accelerations.y;
    zsoma += accelerations.z;
    somaOx += Ox;
    somaOy += Oy;
    somaOz += Oz;

    deviacaocumulativax += (Ox * L) * (M_PI / 180);
    deviacaocumulativay += (Oy * L) * (M_PI / 180);
    deviacaocumulativaz += (Oz * L) * (M_PI / 180);
  }

  float xraw = xsoma / nummedidas;
  float yraw = ysoma / nummedidas;
  float zraw = zsoma / nummedidas;
  float Oxraw = somaOx / nummedidas;
  float Oyraw = somaOy / nummedidas;
  float Ozraw = somaOz / nummedidas;

  Serial.print("Accelerations: X: "); Serial.print(xraw);
  Serial.print(" g, Y: "); Serial.print(yraw);
  Serial.print(" g, Z: "); Serial.print(zraw); Serial.println(" g");

  Serial.print("Angles : OX: "); Serial.print(Oxraw);
  Serial.print(" degrees, OY: "); Serial.print(Oyraw);
  Serial.print(" degrees, OZ: "); Serial.print(Ozraw); Serial.println(" degrees");

  Serial.print("Deviation : DX: "); Serial.print(deviacaocumulativax);
  Serial.print(" meters, DY: "); Serial.print(deviacaocumulativay);
  Serial.print(" meters, DZ: "); Serial.print(deviacaocumulativaz); Serial.println(" meters");

  Serial.println("Cálculo de nova média");

  return;
}


void loop() {
 lermedidas();
 delay(1000);
  
 
}
