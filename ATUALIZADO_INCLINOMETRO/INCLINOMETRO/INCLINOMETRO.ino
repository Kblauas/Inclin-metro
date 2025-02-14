#include <PL_ADXL355.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#define SIZE 500 

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
// adxl355.disableMeasurement();
// adxl355.enableMeasurement();
// float adxl_read();
// float mpu_read();

int con = 0;

float media_adxl_x , media_adxl_y, media_adxl_z, media_mpu_x, media_mpu_y, media_mpu_z, media_mpu_gx, media_mpu_gy, media_mpu_gz;

float a_pitch, a_roll;

void mean_read(){

  float soma_x, soma_y, soma_z, soma_mx, soma_my, soma_mz, soma_gx, soma_gy, soma_gz;
  
  auto accelerations = adxl355.getAccelerations();

  float adxl_x = accelerations.x;
  float adxl_y = accelerations.y;
  float adxl_z = accelerations.z;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float mpu_x = a.acceleration.x;
  float mpu_y = a.acceleration.y;
  float mpu_z = a.acceleration.z;
  float mpu_gx = g.gyro.x;
  float mpu_gy = g.gyro.y ;
  float mpu_gz = g.gyro.z;

  // Serial.println(adxl_x);
  // Serial.println(adxl_y);

  for(int i=0; i <SIZE; i++){
    
    soma_x += adxl_x;
    soma_y += adxl_y;
    soma_z += adxl_z;
    soma_mx += mpu_x;
    soma_my += mpu_y;
    soma_mz += mpu_z;
    soma_gx += mpu_gx;
    soma_gy += mpu_gy;
    soma_gz += mpu_gz;
  }

  media_adxl_x = soma_x / SIZE;
  media_adxl_y = soma_y / SIZE;
  media_adxl_z = soma_z / SIZE; 
  media_mpu_x = soma_mx / SIZE;
  media_mpu_y = soma_my / SIZE; 
  media_mpu_z = soma_mz / SIZE;
  media_mpu_gx = soma_gx / SIZE;
  media_mpu_gy = soma_gy / SIZE;
  media_mpu_gz = soma_gz / SIZE;
  // Serial.println(media_adxl_x);
  // Serial.println(media_adxl_y);

}

void calculos(float ax, float ay, float az){
  a_pitch = atan2(ax, (sqrt(ay * ay)) + (sqrt(az * az))) * 180 / M_PI;
  a_roll = atan2(ay, (sqrt(ax * ax)) + (sqrt(az * az))) * 180 / M_PI;
    

}
void loop() {
  // Read and print the accelerations

  mean_read();
  calculos(media_adxl_x, media_adxl_y, media_adxl_z);
  Serial.println(a_pitch);
  Serial.println(a_roll);

  Serial.println("================================================================");
  delay(250);

}
