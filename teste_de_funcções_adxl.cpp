#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PL_ADXL355.h>
#include <math.h>

// Initialize our sensors
Adafruit_MPU6050 mpu;
PL::ADXL355 adxl355;

// ADXL355 configuration
uint8_t i2cAddress = 0x53;
auto range = PL::ADXL355_Range::range2g;

// Variables for sensor offsets (calibration)
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Variables for orientation calculation
float pitch = 0.0;
float roll = 0.0;
float a_pitch = 0.0;
float a_roll = 0.0;

// Constants for calculations
const float kalman_gain = 0.28;
const float l = 0.5;
const int INTERVALO_LEITURA = 1;  // Reading interval in milliseconds

// Variables for averaging
const int AVERAGE_COUNT = 50;  // Number of readings to average
float mpu_ax_sum = 0, mpu_ay_sum = 0, mpu_az_sum = 0;
float mpu_gx_sum = 0, mpu_gy_sum = 0;
float adxl_ax_sum = 0, adxl_ay_sum = 0, adxl_az_sum = 0;
float pitch_sum = 0, roll_sum = 0;
float a_pitch_sum = 0, a_roll_sum = 0;
int reading_count = 0;

void calcularOffset() {
  Serial.println("Calculating offset...");
  const int numLeituras = 100;
  sensors_event_t a, g, temp;

  // Take multiple readings to establish baseline
  for (int i = 0; i < numLeituras; i++) {
    mpu.getEvent(&a, &g, &temp);
    ax_offset += a.acceleration.x;
    ay_offset += a.acceleration.y;
    az_offset += a.acceleration.z;
    gx_offset += g.gyro.x;
    gy_offset += g.gyro.y;
    gz_offset += g.gyro.z;
    delay(5);
  }

  // Calculate average offsets
  ax_offset /= numLeituras;
  ay_offset /= numLeituras;
  az_offset /= numLeituras;
  gx_offset /= numLeituras;
  gy_offset /= numLeituras;
  gz_offset /= numLeituras;

  Serial.println("Offset calculated!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 37);

  Serial.println("Initializing...");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully!");

  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  // Initialize ADXL355
  adxl355.beginI2C(i2cAddress);
  adxl355.setRange(range);
  adxl355.enableMeasurement();

  // Verify ADXL355 connection
  auto deviceInfo = adxl355.getDeviceInfo();
  if (deviceInfo.deviceId != 0xED) {
    Serial.println("Could not find ADXL355!");
    while (1);
  }
  Serial.println("ADXL355 initialized successfully!");

  // Calculate initial offsets
  calcularOffset();
}

void calcularPosicao(float x, float y, float z, float gx, float gy, float dt, float ax, float ay, float az) {
  dt = dt / 1000;  // Convert milliseconds to seconds
  
  // Update angles using gyroscope data
  pitch += gy * dt;
  roll += gx * dt;

  // Calculate angles from accelerometer data
  float pitch_accel = degrees(atan2(x, sqrt(y * y + z * z))); 
  float roll_accel = degrees(atan2(y, sqrt(x * x + z * z))); 

  // Combine using Kalman filter
  pitch = (1 - kalman_gain) * pitch + kalman_gain * pitch_accel;
  roll = (1 - kalman_gain) * roll + kalman_gain * roll_accel;

  // Calculate angles from ADXL355 data
  a_pitch = atan2(ax, (sqrt(ay * ay)) + (sqrt(az * z))) * 180 / M_PI;
  a_roll = atan2(ay, (sqrt(ax * ax)) + (sqrt(az * az))) * 180 / M_PI;
}

void printAverages() {
  // Calculate and print averages
  Serial.println("=== AVERAGED READINGS ===");
  
  // MPU6050 averages
  Serial.println("MPU6050 Averages:");
  Serial.print("Accel: X="); Serial.print(mpu_ax_sum / AVERAGE_COUNT);
  Serial.print(" Y="); Serial.print(mpu_ay_sum / AVERAGE_COUNT);
  Serial.print(" Z="); Serial.println(mpu_az_sum / AVERAGE_COUNT);
  Serial.print("Gyro: X="); Serial.print(mpu_gx_sum / AVERAGE_COUNT);
  Serial.print(" Y="); Serial.println(mpu_gy_sum / AVERAGE_COUNT);

  // ADXL355 averages
  Serial.println("ADXL355 Averages:");
  Serial.print("Accel: X="); Serial.print(adxl_ax_sum / AVERAGE_COUNT);
  Serial.print(" Y="); Serial.print(adxl_ay_sum / AVERAGE_COUNT);
  Serial.print(" Z="); Serial.println(adxl_az_sum / AVERAGE_COUNT);

  // Angle averages
  Serial.println("Average Angles:");
  Serial.print("Pitch="); Serial.print(pitch_sum / AVERAGE_COUNT);
  Serial.print(" Roll="); Serial.println(roll_sum / AVERAGE_COUNT);
  Serial.print("ADXL Pitch="); Serial.print(a_pitch_sum / AVERAGE_COUNT);
  Serial.print(" ADXL Roll="); Serial.println(a_roll_sum / AVERAGE_COUNT);
  Serial.println("--------------------");

  // Reset sums for next batch of readings
  mpu_ax_sum = 0; mpu_ay_sum = 0; mpu_az_sum = 0;
  mpu_gx_sum = 0; mpu_gy_sum = 0;
  adxl_ax_sum = 0; adxl_ay_sum = 0; adxl_az_sum = 0;
  pitch_sum = 0; roll_sum = 0;
  a_pitch_sum = 0; a_roll_sum = 0;
  reading_count = 0;
}

void loop() {
  static unsigned long ultima_leitura = 0;
  unsigned long tempo_atual = millis();

  if (tempo_atual - ultima_leitura >= INTERVALO_LEITURA) {
    ultima_leitura = tempo_atual;

    // Read from ADXL355
    auto accelerations = adxl355.getAccelerations();
    
    // Read from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply offsets to MPU6050 readings
    float ax = a.acceleration.x - ax_offset;
    float ay = a.acceleration.y - ay_offset;
    float az = a.acceleration.z - az_offset + 9.81;  // Add gravity compensation
    float gx = g.gyro.x - gx_offset;
    float gy = g.gyro.y - gy_offset;
    ax=ax/9.81;
    ay=ay/9.81;
    az=az/9.81;

    // Calculate position and orientation
    calcularPosicao(ax, ay, az, gx, gy, INTERVALO_LEITURA, 
                    accelerations.x, accelerations.y, accelerations.z);

    // Accumulate values for averaging
    mpu_ax_sum += ax;
    mpu_ay_sum += ay;
    mpu_az_sum += az;
    mpu_gx_sum += gx;
    mpu_gy_sum += gy;
    adxl_ax_sum += accelerations.x;
    adxl_ay_sum += accelerations.y;
    adxl_az_sum += accelerations.z;
    pitch_sum += pitch;
    roll_sum += roll;
    a_pitch_sum += a_pitch;
    a_roll_sum += a_roll;
    
    reading_count++;

    // When we have enough readings, print the averages
    if (reading_count >= AVERAGE_COUNT) {
      printAverages();
      delay(100);  // Add small delay to make output readable
    }
  }
}
