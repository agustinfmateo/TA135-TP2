// control clasico

//-------------------------Librerias---------------------------------
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//-------------------------Funciones---------------------------------
void controlP(float anguloMedido);
void controlPI_BD(float anguloMedido, float* ang_error_acum);
void controlPD_BD(float anguloMedido, float* ang_error_ant);
void controlPD_T(float anguloMedido, float* ang_error_ant, float* ang_error_acum);
void controlPI_T(float anguloMedido, float* ang_error_ant, float* ang_error_acum);
void controlPI_tustin(float anguloMedido, float* ang_error_ant, float* accion_ant);
void controlPID_euler(float anguloMedido, float* ang_error_ant, float* accion_ant, float* ang_error_ant2);
void calibrarIMU();
void calibrar();
void moverServo(float theta, int pin);
void matlab_send(float dato1, float dato2, float dato3);

//-------------------------Variables---------------------------------
//Pines
const int servoPin = 17;
const int potPin = 35;

//Variables Pote
int potMin = 4095;
int potMax = 0;
int sensorValue = 0;
int angle = 0;

//Variables Servo
const int servo_resolution = 16;
const int servo_freq = 50; //El servo MG996R usa pwm de 50Hz
const int servo_periodo_ms = 1000/servo_freq;

const int angleMin = 60;
const int angleMax = 120;
float pulsoMin_ms = 0.5; // 0.5ms para 0°
float pulsoMax_ms = 2.45; // 2.45ms para 180°
long dutyMin = (pulsoMin_ms/servo_periodo_ms)*65535; //65535 es porque use 16 bits de precision
long dutyMax = (pulsoMax_ms/servo_periodo_ms)*65535;

//Variables IMU
Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos
unsigned long tiempoMuestra = 0;

const float angServoEq = 90;

float theta0 = 0;
float gyroX_offset = 0;

//Variables Control
float theta   = 0;          //theta actual
float theta_1 = 0;          //theta -1
float u = 0;                //u actual
float u_1=0;                //u-1 anterior
float u_print=0;            //para enviar a imprimir

const float T = 0.02;
const float Ts = 0.02;

float ang_error_act = 0;
float ang_error_ant = 0;
float ang_error_ant2 = 0;

float ang_error_acum = 0;

float accion_actual = 0;
float accion_ant   = 0;

//float angMedido_ant = 0;

const float k_p = -0.588;
const float k_i = -5.6;
const float k_d = -0.0028;

const int zona_muerta = 2;

float angulo = 0;

//--------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  //--------------------Configuracion del Pote-------------------------------------
  pinMode(potPin, INPUT);
  analogReadResolution(12);
  //--------------------Configuracion del Servo-------------------------------------
  ledcAttach(servoPin, servo_freq, servo_resolution); //Pone el pin en PWM
  
  //------------------------Inicializacion de la IMU--------------------------------
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }
  Serial.println("MPU6050 Found!");
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(50);
  //-------------------------Calibracion---------------------------------
  calibrar();
  delay(1000);
  tiempoMuestra = millis();
}

void loop() {
    sensors_event_t a, g, temp;
    unsigned long tiempoActual = millis();

    if(tiempoActual - tiempoMuestra >= periodoMuestreo)
    {
      //Mido el pote
      sensorValue = analogRead(potPin);
      angulo = map(sensorValue, potMin, potMax, angleMin, angleMax); 

      //Mido IMU
      mpu.getEvent(&a, &g, &temp);
      float gyrox = g.gyro.x - gyroX_offset;
      float dt = (tiempoActual - tiempoMuestra) / 1000.0f;
      float anguloRotG = anguloRotAnt + gyrox*dt;
      float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
      float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;

      anguloRotAnt = anguloRot; 
      theta = (anguloRot*RAD_TO_DEG-theta0);  // theta0 el error sistematico de la IMU

      //controlP(theta);
      //controlPD_T(theta, &ang_error_ant, &ang_error_acum);
      //controlPD_BD(theta, &ang_error_ant); //use este 
      //controlPI_T(theta, &ang_error_ant, &ang_error_acum);
      //controlPI_tustin(theta, &ang_error_ant, &accion_ant);
      controlPID_euler(theta, &ang_error_ant, &accion_ant, &ang_error_ant2);

      //Serial.println(theta);
   
      //matlab_send(orden, angulo , theta);
    
      tiempoMuestra = tiempoActual;
    }
}

//-------------------Funciones Control-------------------------------
void controlP(float anguloMedido)
{
 ang_error_act = -anguloMedido;
  if(abs(ang_error_act) < zona_muerta) ang_error_act = 0;
  float accionP = ang_error_act * k_p;
  moverServo(accionP + angServoEq, servoPin);

  matlab_send(accionP + angServoEq, angulo , theta);
}

void controlPD_BD(float anguloMedido, float* ang_error_ant)
{
 ang_error_act = -anguloMedido;
  float accionPD = k_p*ang_error_act + k_d*(ang_error_act - *ang_error_ant)/T;
  *ang_error_ant = ang_error_act;
  if(abs(ang_error_act) < zona_muerta) {
    ang_error_act = 0;
    accionPD = 0;
    matlab_send(accionPD, angulo , theta);
    return;
  }
  moverServo(accionPD + angServoEq, servoPin);

  matlab_send(accionPD, angulo , theta);
}

void controlPD_T(float anguloMedido, float* ang_error_ant, float* ang_error_acum)
{
  ang_error_act = -anguloMedido;
  float D = 2*(ang_error_act - *ang_error_ant)/T - *ang_error_acum;
  float accionPD = k_p*ang_error_act + k_d*D;

  if(abs(ang_error_act) < zona_muerta) {
    ang_error_act = 0;
    accionPD = 0;
    matlab_send(accionPD, angulo , theta);
    ang_error_act = 0;
    return;
  }

  *ang_error_ant = ang_error_act;
  *ang_error_acum = D;
  moverServo(accionPD + angServoEq, servoPin);

  matlab_send(accionPD, angulo , theta);
}

void controlPI_BD(float anguloMedido, float* ang_error_acum)
{
 ang_error_act = -anguloMedido;
  float accionPI = (k_p + k_i*T)*ang_error_act + k_i*(*ang_error_acum);
  *ang_error_acum += ang_error_act;
  if(abs(ang_error_act) < zona_muerta) ang_error_act = 0;
  moverServo(accionPI + angServoEq, servoPin);

  matlab_send(accionPI, angulo , theta);
}

void controlPI_T(float anguloMedido, float* ang_error_ant, float* ang_error_acum)
{
 ang_error_act = -anguloMedido;

  if(abs(ang_error_act) < zona_muerta) ang_error_act = 0;

  float limit_max = angleMax - angServoEq;
  float limit_min = angleMin - angServoEq;

  float I = (*ang_error_acum) + (T/2.0) * ang_error_act + (T/2.0) * (*ang_error_ant);

  float accionPI = k_p*ang_error_act + k_i*I;

  // Antiwindup para Ki negativo
  if (accionPI >= limit_max && ang_error_act < 0) {
      I = *ang_error_acum;
      accionPI = k_p * ang_error_act + k_i * I;
  }
  else if (accionPI <= limit_min && ang_error_act > 0) {
      I = *ang_error_acum;
      accionPI = k_p * ang_error_act + k_i * I;
  }

  *ang_error_ant = ang_error_act;
  *ang_error_acum = I;
  
  moverServo(accionPI + angServoEq, servoPin);

  matlab_send(accionPI, angulo , theta);
}

void controlPI_tustin(float anguloMedido, float* ang_error_ant, float* accion_ant)
{
  //Hay que ponerle el antiwindup pero no la use
  ang_error_act = -anguloMedido;
  if(abs(ang_error_act) < zona_muerta) ang_error_act = 0;
  float accion_actual = (*accion_ant) +  ang_error_act *(k_p+k_i*T/2)+ (*ang_error_ant)*(k_i*T/2-k_p);
  *ang_error_ant = ang_error_act;
  *accion_ant = accion_actual;
  moverServo(accion_actual + angServoEq, servoPin);

  matlab_send(accion_actual, angulo , theta);
}

void controlPID_euler(float anguloMedido, float* ang_error_ant, float* accion_ant, float* ang_error_ant2)
{
  ang_error_act = -anguloMedido;
  if(abs(ang_error_act) < zona_muerta) ang_error_act = 0;

  float limit_max = angleMax - angServoEq;
  float limit_min = angleMin - angServoEq;

  float accion_actual = (*accion_ant) +  ang_error_act *(k_p+k_i*T+k_d/T) - (*ang_error_ant)*(k_p+2*k_d/T) + (*ang_error_ant2)*k_d/T;

  //Antiwindup
  if (accion_actual > limit_max) {
    accion_actual = limit_max;
  }
  else if (accion_actual < limit_min) {
    accion_actual = limit_min;
  }

  *ang_error_ant2 = *ang_error_ant;
  *ang_error_ant = ang_error_act;
  *accion_ant = accion_actual;
  
  moverServo(accion_actual + angServoEq, servoPin);

  matlab_send(accion_actual, angulo , theta);
}

//-------------------Funcion Calibracion-----------------------------
void calibrarIMU() {
  Serial.println("Calibrando IMU...");
  delay(500);

  float sumatoriaAngulos = 0;
  float sumatoriaGyro = 0;
  int muestras = 500;
  sensors_event_t a, g, temp;

  for (int i = 0; i < muestras; i++) {
    mpu.getEvent(&a, &g, &temp);
    //Calibrar angulo en equilibrio
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z); //Solo el acelerometro pq ta quieto
    sumatoriaAngulos += (anguloRotA * 180.0 / PI);
    //Calibrar offset del giroscopio
    sumatoriaGyro += g.gyro.x;
    delay(20);
  }

  theta0 = sumatoriaAngulos / muestras; //promedio
  gyroX_offset = sumatoriaGyro / muestras;

  anguloRotAnt = theta0 * PI / 180.0; 
  
  Serial.print("Calibracion exitosa, Nuevo theta0: ");
  Serial.print(theta0);
  Serial.print("\t Offset Gyro: ");
  Serial.println(gyroX_offset);
}

void calibrar() {
  calibrarIMU();

  moverServo(angServoEq, servoPin);
  delay(1000);

  Serial.println("Valores antes de Calibrar");
  Serial.println(potMin);
  Serial.println(potMax);

  long suma = 0;
  int muestras = 500;

  moverServo(angleMin, servoPin);
  delay(1000);
  
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(potPin);
    delay(1);
  }
  int potValor = suma/muestras;

  potMax = max(potMax, potValor);
  potMin = min(potMin, potValor);

  moverServo(angleMax, servoPin);
  delay(1000);

  suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(potPin);
    delay(1);
  }
  potValor = suma/muestras;

  potMax = max(potMax, potValor);
  potMin = min(potMin, potValor);

  Serial.println("Valores despues de Calibrar");
  Serial.println(potMin);
  Serial.println(potMax);
  
  moverServo(angServoEq, servoPin);
  delay(2000);
}

//----------------Funciones para movimiento del Servo----------------
void moverServo(float theta, int pin) {
  theta = constrain(theta+6, angleMin, angleMax);
  int dutyCycle = map(theta, 0, 180, dutyMin, dutyMax); //Mapea valores de theta a dutycycle
  ledcWrite(pin, dutyCycle);
}

//-----------------------Envio de datos a MATLAB-------------------------
void matlab_send(float dato1, float dato2, float dato3){
  //Serial.write("abcd");
  Serial.write(97);
  Serial.write(98);
  Serial.write(99);
  Serial.write(100);
  
  byte * b0 = (byte *) &dato1;
  Serial.write(b0,4);
  
  byte * b1 = (byte *) &dato2;
  Serial.write(b1,4);

  byte * b2 = (byte *) &dato3;
  Serial.write(b2,4);
}