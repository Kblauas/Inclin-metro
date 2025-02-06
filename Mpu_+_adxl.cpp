#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PL_ADXL355.h>
#include <math.h>

// Inclui o sensor MPU6050
Adafruit_MPU6050 mpu;

//Inclui o sensor ADXL355
PL::ADXL355 adxl355;
uint8_t i2cAddress = 0x53;
uint8_t I2c_SDAPIN=21;
uint8_t I2c_SCLPIN=37;

// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

HardwareSerial mySerial(1);
#define RS485_CONTROL_PIN 3
#define INTERVALO_LEITURA 1

float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

bool sistemaAtivo = false;
bool salvarDados = true; // Variável para controlar se as leituras devem ser salvas

int p = 0;
int r = 0;
int pos = 0;

float pitch = 0.0;
float roll = 0.0;
float position = 0.0; // Posição acumulada
const float kalman_gain = 0.28; // Exemplo de ganho de Kalman
const float l = 0.5;

unsigned long ultima_leitura = 0;

// Listas para armazenar dados
#define TAM_MAX 5000
#define BUFFER_SIZE 100
float lista_pitch[TAM_MAX];
float lista_roll[TAM_MAX];
float lista_position[TAM_MAX];
int lista_count = 0; // Contador de leituras salvas

// Buffers circulares para média móvel
float buffer_ax[BUFFER_SIZE] = {0};
float buffer_ay[BUFFER_SIZE] = {0};
float buffer_az[BUFFER_SIZE] = {0};
float buffer_gx[BUFFER_SIZE] = {0};
float buffer_gy[BUFFER_SIZE] = {0};
int buffer_index = 0;

void calcularOffset() {
  Serial.println("Calculando offset...");
  const int numLeituras = 100;
  sensors_event_t a, g, temp;

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

  ax_offset /= numLeituras;
  ay_offset /= numLeituras;
  az_offset /= numLeituras;
  gx_offset /= numLeituras;
  gy_offset /= numLeituras;
  gz_offset /= numLeituras;

  Serial.println("Offset calculado!");
}

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 5, 4);
  Serial.begin(115200);
  pinMode(RS485_CONTROL_PIN, OUTPUT);
  digitalWrite(RS485_CONTROL_PIN, LOW);

  Wire.begin(21, 37);

  Serial.println("Iniciando...");

  // Inicialização do sensor MPU6050
  if (!mpu.begin()) {
    Serial.println("Não foi possível encontrar o MPU6050!");
    while (1); // Travar aqui se o sensor não for encontrado
  }
 

  Serial.println("MPU6050 inicializado com sucesso!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  //inicialização do adxl
  adxl355.beginI2C(i2cAddress);
  // Set ADXL355 range
  adxl355.setRange(range);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);

  auto deviceInfo=adxl355.getDeviceInfo();
  if(deviceInfo.deviceId!=0xED){
    Serial.println("Não foi possível encontrar o ADXL355!");
    while(1);
  }

   Serial.println("ADXL355 inicializado com sucesso!");


  // Wait for Serial ready state
  while(!Serial);

  

  calcularOffset();
}

void leituraADXL(){
  int nummedidas = 10;
  float xsoma = 0, ysoma = 0, zsoma = 0;
  float somaOx = 0, somaOy = 0, somaOz = 0;
  float deviacaocumulativax = 0, deviacaocumulativay = 0, deviacaocumulativaz = 0;
  Serial.println("Resultados ADXL");

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

    deviacaocumulativax += (Ox * l) * (M_PI / 180);
    deviacaocumulativay += (Oy * l) * (M_PI / 180);
    deviacaocumulativaz += (Oz * l) * (M_PI / 180);
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




void calcularPosicao(float x, float y, float z, float gx, float gy, float dt) {
  dt = dt / 1000;
  pitch += gy * dt;
  roll += gx * dt;

  float pitch_accel = degrees(atan2(x, sqrt(y * y + z * z))); 
  float roll_accel = degrees(atan2(y, sqrt(x * x + z * z))); 

  pitch = (1 - kalman_gain) * pitch + kalman_gain * pitch_accel;
  roll = (1 - kalman_gain) * roll + kalman_gain * roll_accel;

  float posicao = sin(radians(pitch)) * l;
  position += posicao;
}

void atualizarBuffers(float ax, float ay, float az, float gx, float gy) {
  buffer_ax[buffer_index] = ax;
  buffer_ay[buffer_index] = ay;
  buffer_az[buffer_index] = az;
  buffer_gx[buffer_index] = gx;
  buffer_gy[buffer_index] = gy;

  buffer_index = (buffer_index + 1) % BUFFER_SIZE;
}

void calcularMedia(float &ax, float &ay, float &az, float &gx, float &gy) {
  float soma_ax = 0, soma_ay = 0, soma_az = 0, soma_gx = 0, soma_gy = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    soma_ax += buffer_ax[i];
    soma_ay += buffer_ay[i];
    soma_az += buffer_az[i];
    soma_gx += buffer_gx[i];
    soma_gy += buffer_gy[i];
  }
  ax = soma_ax / BUFFER_SIZE;
  ay = soma_ay / BUFFER_SIZE;
  az = soma_az / BUFFER_SIZE;
  gx = soma_gx / BUFFER_SIZE;
  gy = soma_gy / BUFFER_SIZE;
}

void enviarDadosRS485() {
  Serial.println("Enviando dados acumulados via RS485...");
  digitalWrite(RS485_CONTROL_PIN, HIGH);
  for (int i = 0; i < lista_count; i++) {
 // Cabeçalho

    // int p = (int)(lista_pitch[i] * 100);
    // int r = (int)(lista_roll[i] * 100);
    // int pos = (int)(lista_position[i] * 100);
    p += lista_pitch[i];
    r += lista_roll[i];
    pos += lista_position[i];

    

  }
  p = p / lista_count;
  r = r / lista_count;
  pos = lista_count;

  

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
  lista_count = 0; // Limpa a lista após o envio
}

void verificarComandoRS485() {
  if (mySerial.available()) {
    String comando = "";
    while (mySerial.available()) {
      char c = mySerial.read();
      comando += c;
    }
    if (comando == "P") {
      sistemaAtivo = false;
      salvarDados = false; // Parar de salvar dados
      enviarDadosRS485();
      Serial.println("Sistema pausado!");
    } else if (comando == "R") {
      sistemaAtivo = true;
      salvarDados = true; // Retomar salvamento
      Serial.println("Sistema retomado!");
    } 
  }
}

void loop() {
  unsigned long tempo_atual = millis();

  verificarComandoRS485();

  if (sistemaAtivo && salvarDados && lista_count < TAM_MAX) {
    if (tempo_atual - ultima_leitura >= INTERVALO_LEITURA) {
      ultima_leitura = tempo_atual;

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float ax = a.acceleration.x - ax_offset;
      float ay = a.acceleration.y - ay_offset;
      float az = a.acceleration.z - az_offset + 9.81;
      float gx = g.gyro.x - gx_offset;
      float gy = g.gyro.y - gy_offset;

      atualizarBuffers(ax, ay, az, gx, gy);
      calcularMedia(ax, ay, az, gx, gy);
      calcularPosicao(ax, ay, az, gx, gy, INTERVALO_LEITURA);
      
      Serial.print(p);
      Serial.print(" --- " );
      Serial.print(r );
      Serial.print(" --- ");
      Serial.println(pos );
      // Salvar dados nas listas
      lista_pitch[lista_count] = pitch;
      lista_roll[lista_count] = roll;
      lista_position[lista_count] = position;
      lista_count++;

      leituraADXL();
    }
  }
}
