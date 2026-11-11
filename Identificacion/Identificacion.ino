// Identificación

//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//-------------------------Variables---------------------------------

const int servoPin = 17;  //VERIFICAR
const int potPin = 35;

//Variables Pote
int potMin = 4095;
int potMax = 0;
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

const float angServoEq = angServoEq;

float theta0 = 0;
float gyroX_offset = 0;

float theta   = 0;               //theta actual

const float T = 0.02;
const float Ts = 0.02;

static unsigned long tiempoUltimoEscalon = 0;
int comandoServo = 90;

int d = 0;

//--------------------------------------------------------------------

void setup() {
  //--------------------Configuracion del Pote-------------------------------------
  pinMode(potPin, INPUT);
  analogReadResolution(12);
  //--------------------Configuracion del Servo-------------------------------------
  ledcAttach(servoPin, servo_freq, servo_resolution); //Pone el pin en PWM
  Serial.begin(115200);
  //------------------------Inicializacion de la IMU--------------------------------
  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(50);
  //-------------------------Calibracion---------------------------------
  calibrar();
  delay(8000);
  tiempoMuestra = millis();
}

void loop() {
    sensors_event_t a, g, temp;

    unsigned long tiempoActual = millis();

    if(tiempoActual - tiempoMuestra >= periodoMuestreo)
    {
      //Mido el pote
      int potValor = analogRead(potPin);
      float angulo = map(potValor, potMin, potMax, angleMin, angleMax); 

      //Mido IMU
      mpu.getEvent(&a, &g, &temp);
      float gyrox = g.gyro.x - gyroX_offset;
      float dt = (tiempoActual - tiempoMuestra) / 1000.0f;
      float anguloRotG = anguloRotAnt + gyrox*dt;
      float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
      float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;
      
      float anguloMedido = anguloRot*180/PI;

      anguloRotAnt = anguloRot;
      theta = (anguloRot*180/PI-theta0);// theta0 el error sistematico de la IMU

      if (tiempoActual - tiempoUltimoEscalon >= 7000) {
        d++;
        if (d % 2 == true) {
          comandoServo = angleMin;
        }
        else{
          comandoServo = angleMax;
        }
        moverServo(comandoServo, servoPin);
        tiempoUltimoEscalon = tiempoActual;
      }

      //Enviamos el comando, el angulo del pote y el angulo de la imu
      matlab_send(comandoServo, angulo, theta);
      //Serial.print("theta");
      //Serial.print(theta);
      //Serial.print("\t phi");
      //Serial.println(angulo);

      tiempoMuestra = tiempoActual;
    }
}

//-------------------Funcion Calibracion-----------------------------
void calibrarIMU() {
  Serial.println("TCHH DEJA QUIETO AHI");
  delay(1000);

  float sumatoriaAngulos = 0;
  float sumatoriaGyro = 0;
  int muestras = 500;
  sensors_event_t a, g, temp;

  Serial.println("NO TOQUES NADA CARAJO");
  for (int i = 0; i < muestras; i++) {
    mpu.getEvent(&a, &g, &temp);
    //Calibrar angulo en equilibrio
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z); //Solo el acelerometro pq ta quieto
    sumatoriaAngulos += (anguloRotA * 180.0 / PI);
    //Calibrar offset del giroscopio
    sumatoriaGyro += g.gyro.x;
    delay(10);
  }

  theta0 = sumatoriaAngulos / muestras; //promedio
  gyroX_offset = sumatoriaGyro / muestras;

  anguloRotAnt = theta0 * PI / 180.0; 
  
  Serial.print("Calibracion exitosa! Nuevo theta0: ");
  Serial.print(theta0);
  Serial.print("\t Offset Gyro: ");
  Serial.println(gyroX_offset);
}

void calibrar() {
  calibrarIMU();

  moverServo(angServoEq, servoPin);
  delay(10000);

  Serial.println("Valores antes de Calibrar");
  Serial.println(potMin);
  Serial.println(potMax);

  long suma = 0;
  int muestras = 500;

  moverServo(angleMin, servoPin);
  delay(3000);
  
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(potPin);
    delay(1);
  }
  int potValor = suma/muestras;

  potMax = max(potMax, potValor);
  potMin = min(potMin, potValor);

  moverServo(angleMax, servoPin);
  delay(3000);

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
  delay(3000);
}

//----------------Funciones para movimiento del Servo----------------
void moverServo(int theta, int pin) {
  theta = constrain(theta, angleMin, angleMax);
  int dutyCycle = map(theta+6, 0, 180, dutyMin, dutyMax); //Mapea valores de theta a dutycycle
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
