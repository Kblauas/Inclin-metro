#include <PL_ADXL355.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


float adxl_read(){  
  auto accelerations = adxl355.getAccelerations();
  
  Serial.print("Accelerations: X: ");
  float adxl_x = accelerations.x;
  Serial.print(adxl_x);
  Serial.print(" g, Y: ");
  float adxl_y = accelerations.y;
  Serial.print(adxl_y);
  Serial.print(" g, Z: ");
  float adxl_z = accelerations.z;
  Serial.print(adxl_z);
  Serial.println(" g");

  return adxl_x, adxl_y, adxl_z;
}

float mpu_read(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  float mpu_x = a.acceleration.x;
  Serial.print(mpu_x);
  Serial.print(", Y: ");
  float mpu_y = a.acceleration.y;
  Serial.print(mpu_y);
  Serial.print(", Z: ");
  float mpu_z = a.acceleration.z;
  Serial.print(mpu_z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  float mpu_gx = g.gyro.x;
  Serial.print(mpu_gx);
  Serial.print(", Y: ");
  float mpu_gy = g.gyro.y ;
  Serial.print(mpu_gy);
  Serial.print(", Z: ");
  float mpu_gz = g.gyro.z;
  Serial.print(mpu_gz);
  Serial.println(" rad/s");

  return mpu_x, mpu_y, mpu_z, mpu_gx, mpu_gy, mpu_gz;

}