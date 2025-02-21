#include <PL_ADXL355.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#define SIZE 500 

HardwareSerial mySerial(1);
#define RS485_CONTROL_PIN 3
#define INTERVALO_LEITURA 1

enum EstadoSistema {
  PAUSADO,
  EXECUTANDO
};

EstadoSistema estadoAtual = PAUSADO;

void disableMeasurement();

float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

int p = 0;
int r = 0;
int pos = 0;

float pitch = 0.0;
float roll = 0.0;
float position = 0.0; // Posição acumulada
float kalman_gain = 0.28;
float l = 0.5;
int lista_count = 0;
unsigned long ultima_leitura = 0;

PL::ADXL355 adxl355;
//uint8_t spiCsPin = 2;
uint8_t i2cAddress = 0x53;

#define TAM_MAX 5000
float lista_pitch[TAM_MAX];
float lista_roll[TAM_MAX];
float lista_position[TAM_MAX];

Adafruit_MPU6050 mpu;
// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//==============================================================================

void setupMPU();

void setup() {

  setupMPU();
  mySerial.begin(115200, SERIAL_8N1, 5, 4);
  Serial.begin(115200);
  pinMode(RS485_CONTROL_PIN, OUTPUT);
  digitalWrite(RS485_CONTROL_PIN, LOW);
  Wire.begin(21, 37);
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  //adxl355.beginSPI(spiCsPin);
  adxl355.beginI2C(i2cAddress);

  // Set ADXL355 range
  adxl355.setRange(range);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  
  // Wait for Serial ready state
  while(!Serial);
}

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

void verificarComandoRS485() {
  if (mySerial.available()) {
    String comando = "";
    while (mySerial.available()) {
      char c = mySerial.read();
      comando += c;
    }
    if (comando == "P") {
      estadoAtual = PAUSADO;
      adxl355.disableMeasurement();
      
      Serial.println("Sistema pausado!");
    } else if (comando == "R") {
      estadoAtual = EXECUTANDO;
      adxl355.enableMeasurement();
      Serial.println("Sistema retomado!");
    }
  }
}

void enviarDadosRS485() {
  Serial.println("Enviando dados acumulados via RS485...");
  digitalWrite(RS485_CONTROL_PIN, HIGH);
  int L = 0.5;
  for (int i = 0; i < lista_count; i++) {
 // Cabeçalho

    // int p = (int)(lista_pitch[i] * 100);
    // int r = (int)(lista_roll[i] * 100);
    // int pos = (int)(lista_position[i] * 100);
    p += lista_pitch[i];
    r += lista_roll[i];
    //pos += lista_position[i];

    

  }
  p = p / lista_count;
  r = r / lista_count;
  pos = sin(p)*L;

  
  if(estadoAtual ==PAUSADO){
    byte buffer[7];
    buffer[0] = 0xAA;
    buffer[1] = (p >> 8) & 0xFF;
    buffer[2] = p & 0xFF;
    buffer[3] = (r >> 8) & 0xFF;
    buffer[4] = r & 0xFF;
    buffer[5] = (pos >> 8) & 0xFF;
    buffer[6] = pos & 0xFF;

    mySerial.write(buffer, 7);
    mySerial.flush();
    //}
    digitalWrite(RS485_CONTROL_PIN, LOW);

    Serial.println("Dados enviados!");
  }
  lista_count = 0; // Limpa a lista após o envio
}

void loop() {
  // Read and print the accelerations
  verificarComandoRS485();
  if (estadoAtual == EXECUTANDO && lista_count < TAM_MAX) {
    unsigned long tempo_atual = millis();
    if (tempo_atual - ultima_leitura >= INTERVALO_LEITURA) {
      ultima_leitura = tempo_atual;
      mean_read();
      calculos(media_adxl_x, media_adxl_y, media_adxl_z);
      Serial.println(a_pitch);
      Serial.println(a_roll);

      lista_pitch[lista_count] = pitch;
      lista_roll[lista_count] = roll;
      //lista_position[lista_count] = position;
      lista_count++;
    }
  }
  enviarDadosRS485();

  Serial.println("==========================================================================");
 

}
