#include <Arduino.h>
#include <PL_ADXL355.h>
#include <math.h>




//class AccelerometerFilter {
//private:
//    float x_est = 0;  
//    float P = 1.0;    
//    float Q = 0.1;    
//    float R = 0.1;    

//    static const int MA_WINDOW = 10;
//    float maBuffer[MA_WINDOW] = {0};
//    int maIndex = 0;
//    float maSum = 0;

//public:
//    float kalmanUpdate(float measurement) {
//        P = P + Q;
        
//        float K = P / (P + R);
//        x_est = x_est + K * (measurement - x_est);
//        P = (1 - K) * P;
        
//        return x_est;
//    }

//    float movingAverageUpdate(float newValue) {
//        maSum = maSum - maBuffer[maIndex] + newValue;
//        maBuffer[maIndex] = newValue;
        
//        maIndex = (maIndex + 1) % MA_WINDOW;
        
//        return maSum / MA_WINDOW;
//    }

//    float filterAcceleration(float rawAccel) {
//        float maFiltered = movingAverageUpdate(rawAccel);
//        return kalmanUpdate(maFiltered);
//    }
//};


// put function declarations here:
PL::ADXL355 adxl355;
//uint8_t spiCsPin = 2;
uint8_t i2cAddress = 0x1D;

float L=0.5;
auto activityAxes = PL::ADXL355_Axes::x | PL::ADXL355_Axes::y | PL::ADXL355_Axes::z;
// Activity detection threshold, g
float activityThreshold = 1.5;
// Activity detection count
uint8_t activityCount = 3;

const uint8_t interruptPin = 12; // Choose an appropriate interrupt pin
volatile bool interruptTriggered = false;


// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//AccelerometerFilter filterX, filterY, filterZ;

// Velocity and Position tracking
//float velocityX = 0, velocityY = 0, velocityZ = 0;
//float positionX = 0, positionY = 0, positionZ = 0;
//float previousAccelX = 0, previousAccelY = 0, previousAccelZ = 0;


float scaleFactor = adxl355.getAccelerationScaleFactor();

void IRAM_ATTR handleInterrupt() {
  interruptTriggered = true;
}


void setup() {
  adxl355.beginI2C(i2cAddress);

  adxl355.setRange(range);

  adxl355.enableMeasurement();

  adxl355.setOutputDataRate(PL::ADXL355_OutputDataRate::odr2000);

    

  Serial.begin(115200);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

  adxl355.setActivityDetectionThreshold(activityThreshold);
  adxl355.setActivityDetectionCount(activityCount);
  adxl355.setActivityDetectionAxes(activityAxes);


  
  while(!Serial);

    


}


//void updatePositionAndVelocity(float accelX, float accelY, float accelZ, float dt) {
    // Calibration and filtering
  //float accelXFiltered = filterX.filterAcceleration(accelX);
  //float accelYFiltered = filterY.filterAcceleration(accelY);
  //float accelZFiltered = filterZ.filterAcceleration(accelZ);

    // Velocity integration with trapezoidal rule and drift reduction
  //velocityX += (previousAccelX + accelXFiltered) * dt / 2.0;
  //velocityY += (previousAccelY + accelYFiltered) * dt / 2.0;
  //velocityZ += (previousAccelZ + accelZFiltered) * dt / 2.0;

    // Slight velocity damping to reduce drift
  //velocityX *= 0.99;
  //velocityY *= 0.99;
  //velocityZ *= 0.99;

    // Position integration
  //positionX += velocityX * dt;
  //positionY += velocityY * dt;
  //positionZ += velocityZ * dt;

    // Update previous acceleration for next iteration
  //previousAccelX = accelXFiltered;
  //previousAccelY = accelYFiltered;
  //previousAccelZ = accelZFiltered;
//}


void loop() {
  int i=0;
  int leituras=51;
  float deviacaox=0, deviacaoy=0, deviacaoz=0;
  float deviacaocumulativax=0, deviacaocumulativay=0, deviacaocumulativaz;
  float Ox, Oy, Oz, pitch, roll, yaw;
  unsigned long t_final, t_inicial;
  float tempoDemedidas=1;
  t_inicial=millis();
    
  for(i=0;i<leituras;i++){
        
      
      
    //void setOffsets(PL::ADXL355_Accelerations offsets);

    auto accelerations = adxl355.getAccelerations();
    auto offsets = adxl355.getOffsets();
    
    //offsets.x=0.1;
    //offsets.y=0.2;
    //offsets.z=-0.4;
    Serial.print("offsets x = ");;
      
    Serial.print(offsets.x);
    Serial.print(" g , Y = ");
    Serial.print(offsets.y);
    Serial.print(" g , Z = ");
    Serial.print(offsets.z);
    Serial.println("g");


     
      //calibração de cada aceleração
    float calibradox=accelerations.x-offsets.x;
    float calibradoy=accelerations.y-offsets.y;
    float calibradoz=accelerations.z-offsets.z;

    //compensando a gravidade
    //float xcompensar = 1.00;
    //float ycompensar = -0.04;
    //float zcompensar = 0.11;

    //float compensadox=calibradox-xcompensar;
    //float compensadoy=calibradoy-ycompensar;
    //float compensadoz=calibradoz-zcompensar;

     //Modulo da aceleração
    float A=(sqrt((calibradox*calibradox)+(calibradoy*calibradoy)+(calibradoz*calibradoz)));

    float dt;
    t_final=millis();
    dt=t_final-t_inicial;
    dt=dt/1000;
    if(A<=0.5){
      Serial.println("Não há atividade");
      calibradox=0;
      calibradoy=0;
      calibradoz=0;
      A==0;
      
    }
    else{
      Serial.println("Há movimento");
      
    //updatePositionAndVelocity(calibradox, calibradoy, calibradoz, tempoDemedidas);

      //inclinação em cada eixo
      Ox = atan2(calibradox, sqrt(calibradoy * calibradoy + calibradoz * calibradoz)) * 180 / M_PI;
      Oy = atan2(calibradoy, sqrt(calibradox * calibradox + calibradoz * calibradoz)) * 180 / M_PI;
      Oz = atan2(calibradoz, sqrt(calibradox * calibradox + calibradoy * calibradoy)) * 180 / M_PI;
      //cálculo do pitch e do roll, com aproximação do yaw
      pitch = atan2(-calibradox, sqrt(calibradoy*calibradoy+calibradoz*calibradoz))*180/M_PI;
      roll = atan2(calibradoy,calibradoz)*180/M_PI;
      yaw = atan2(sqrt(calibradox*calibradox+calibradoy*calibradoy),calibradoz)*180/M_PI; //aproximado
      
      //Deviação em cada eixo;
      
      //para calcular a inclinação, precisa-se pegar uma medida ereta para setar os valores de aceleração base para cada eixo
      //inclinacao = 180.0*acos( (xo*calibradox + yo*calibradoy + zo*calibradoz)/sqrt( (calibradox*calibradox + calibradoy*calibradoy + calibradoz*calibradoz)*(xo*xo + yo*yo + zo*zo)))/M_PI;

    }
    deviacaox =L*(sin(Ox*M_PI/180));
    deviacaoy =L*(sin(Oy*M_PI/180));
    deviacaoz =L*(sin(Oz*M_PI/180));

    deviacaocumulativax +=L*(sin(Ox*M_PI/180));
    deviacaocumulativay +=L*(sin(Oy*M_PI/180));
    deviacaocumulativaz +=L*(sin(Oz*M_PI/180));

    

    //Serial.print("Filtered Position: X=");
    //Serial.print(positionX, 4);
    //Serial.print(" m, Y=");
    //Serial.print(positionY, 4);
    //Serial.print(" m, Z=");
    //Serial.print(positionZ, 4);
    //Serial.println(" m");
        
    //Serial.print("Velocity: X=");
    //Serial.print(velocityX, 4);
    //Serial.print(" m/s, Y=");
    //Serial.print(velocityY, 4);
    //Serial.print(" m/s, Z=");
    //Serial.print(velocityZ, 4);
    //Serial.println(" m/s");
        
    Serial.print("Aceleração em x = ");
    Serial.print(accelerations.x, 2);
    Serial.print(" g, Y = ");
    Serial.print(accelerations.y, 2);
    Serial.print(" g , Z =");
    Serial.print(accelerations.z, 2);
    Serial.print(" g, magnitude da aceleração = ");
    Serial.print(A, 2);

    //Serial.print("Acelerações compensadas");
    //Serial.print(compensadox);
    //Serial.print(" g, y = ");
    //Serial.print(compensadoy);
    //Serial.print(" g, z = ");
    //Serial.print(compensadoz);
    //Serial.println(" g");

    Serial.print(" g, tempo passado = ");
    Serial.print(dt, 2);
    Serial.print(" segundos");
    Serial.println();

    Serial.print("Inclinação em x = ");
    Serial.print(Ox, 4);
    Serial.print(" graus, Inclinação em y = ");
    Serial.print(Oy,4);
    Serial.print(" graus, Inclinação em z = ");
    Serial.print(Oz, 4);
    Serial.println(" graus");

    Serial.print("deviação da vertical em x = ");
    Serial.print(deviacaox, 4);
    Serial.print(" metros, Y = ");
    Serial.print(deviacaoy, 4);
    Serial.print(" metros, Z = ");
    Serial.print(deviacaoz, 4);
    Serial.println(" metros");

    Serial.print("deviação cumulativa em X = ");
    Serial.print(deviacaocumulativax, 4);
    Serial.print(" metros, Y = ");
    Serial.print(deviacaocumulativay, 4);
    Serial.print(" metros, z = ");
    Serial.print(deviacaocumulativaz, 4);
    Serial.println(" metros");

    Serial.print("Pitch = ");
    Serial.print(pitch, 4);
    Serial.print(" graus, Roll = ");
    Serial.print(roll, 4);
    Serial.print(" graus, Yaw aproximado = ");
    Serial.print(yaw, 4);
    Serial.println(" graus");

    delay(1000);
    if(i==leituras){
      Serial.print("recomeçando. . .");
      i=0;
    }
  }

    
  }

