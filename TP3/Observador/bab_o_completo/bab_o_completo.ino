#include <Adafruit_MPU6050.h>
#include <NewPing.h>
#include <Servo.h>

#define PI 3.14159265359

#define MAX_DELAY_US 16383

#define ADC_BITS 10
#define ADC_MAX  ((2 << ADC_BITS) - 1)

#define BAUD_RATE  115200
#define SEND_COUNT 9

#define CONTROL_PERIOD_US  20000 // 20 ms (50 Hz)
#define DT                 (CONTROL_PERIOD_US / 1e6)

#define BEAM_MID_M 0.180 // Mitad de la longitud de la barra en metros (MEDIR)
#define CART_MID_M 0.028  // Mitad de la longitud del carro (MEDIR)

#define CALIB_ITERS 25 // Medio segundo de calibración

#define SERVO_AMP_DEG 90
#define SERVO_MAX 30
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

#define SONAR_MAX_CM   50
#define SONAR_US_PER_M 5831

#define PIN_POT   A3
#define PIN_SERVO 9
#define PIN_TRIG  6
#define PIN_ECHO  7

#define ALPHA 0.05

#define CALIBRATE 1

unsigned long last_time;

Servo servo;
NewPing sonar(PIN_TRIG, PIN_ECHO, SONAR_MAX_CM);
Adafruit_MPU6050 mpu;

float gyro_bias;
float acc_bias;
float ang_vel;

void setup() {
  Serial.begin(BAUD_RATE);
  servo.attach(PIN_SERVO);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) {
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);

  moveServo(0);

  delay(2000);

#if CALIBRATE
  sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);
  //gyro_bias = g.gyro.x;

  acc_bias = 0;
  for (int i = 0; i < CALIB_ITERS; i++){
    delayMicroseconds(CONTROL_PERIOD_US);
    mpu.getEvent(&a, &g, &temp);
    acc_bias += atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    gyro_bias += g.gyro.x;
  }
  acc_bias /= CALIB_ITERS;
  gyro_bias /= CALIB_ITERS;
#else
  // Valores a mano
  gyro_bias = 0;
  acc_bias  = 0;
#endif

  Serial.print("Gyro bias: ");
  Serial.println(gyro_bias);
  Serial.print("Acc bias:  ");
  Serial.println(acc_bias);

  // Siempre debe ser la última línea del setup
  last_time = micros();
}

void loop() {
  runPeriodicallyMicros(CONTROL_PERIOD_US);

  static float ang = 0;
  static float theta_obs = 0;
  static float omega_obs = 0;
  static float pos_obs = 0;
  static float vel_obs = 0;
  static float pos1 = 0;
  static float vel = 0;
  static float time = 0;
  static float u = 0;
  static float ref = 0;


  // float refs[] = {-15, 10};
  float pos = readPosition();
  vel = (pos- pos1)/DT;
  pos1 = pos;
  ang = estimateAngle(ang);
  time += DT;

  // int idx = (int)(time/.5) % 2;
  if (time < .45){
    ref = -20;
  } else if (time < 1.5) {
    ref = 15;
  } else if (time < 2.5) {
    ref = -15;
  } else {
    ref = 0;
  }
  u=ref;
  moveServo(u);

  const float A[4][4] = {{1.0, 0.02, 0.0, 0.0}, 
                  {-2.75, 0.572, 0.0, 0.0}, 
                  {0.0, 0.0, 1.0, 0.02}, 
                  {-0.003, 0.0, 0.0, 0.965}};
  const float B[4] = {0.0, 1.0728, 0.0, 0.0};
  const float C[2][4] = {{1.0, 0.0, 0.0, 0.0}, 
                  {0.0, 0.0, 1.0, 0.0}};

  const float L[4][2] = {{.6371, 0.0}, {-3.4543, 0.0}, {0.0, 1.0301}, {-0.003, 11.1285}};

  // A*[th om pos vel] + B*u + L*[th-th_o pos-pos_o]
  // Implementamos observador
  float theta_obs1 = A[0][0]*theta_obs + A[0][1]*omega_obs + A[0][2]*pos_obs + A[0][3]*vel_obs + L[0][0]*(ang-theta_obs) + L[0][1]*(pos-pos_obs);
  float omega_obs1 = A[1][0]*theta_obs + A[1][1]*omega_obs + A[1][2]*pos_obs + A[1][3]*vel_obs + B[1]*u + L[1][0]*(ang-theta_obs) + L[1][1]*(pos-pos_obs);
  float pos_obs1   = A[2][0]*theta_obs + A[2][1]*omega_obs + A[2][2]*pos_obs + A[2][3]*vel_obs + L[2][0]*(ang-theta_obs) + L[2][1]*(pos-pos_obs);
  float vel_obs1   = A[3][0]*theta_obs + A[3][1]*omega_obs + A[3][2]*pos_obs + A[3][3]*vel_obs + L[3][0]*(ang-theta_obs) + L[3][1]*(pos-pos_obs);
  theta_obs = theta_obs1;
  omega_obs = omega_obs1;
  pos_obs = pos_obs1;
  vel_obs = vel_obs1;

  float datos[] = {u, pos, pos_obs, vel, vel_obs, ang, theta_obs, ang_vel, omega_obs};
  matlab_send(datos);

  // Serial.print(ref);
  // Serial.print(", ");
  // Serial.print(pos);
  // Serial.print(", ");
  // Serial.print(ang);
  // Serial.println();
}

/// Toma valores entre -90 y 90 grados.
/// Satura valores por fuera de ese rango.
void moveServo(float angle) {
  angle = saturate(angle, -SERVO_MAX, SERVO_MAX);
  unsigned int us = remap(angle, -SERVO_AMP_DEG, SERVO_AMP_DEG, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(us);
}

/// Devuelve la posición del centro del carrito
/// respecto del centro de la barra, en metros.
float readPosition() {
  float distance = readSonar();
  return distance + CART_MID_M - BEAM_MID_M;
}

/// Devuelve una posición de referencia para el carrito
/// respecto del centro de la varilla, en metros.
float readReference() {
  float value = analogRead(PIN_POT);
  return remap(value, 0, ADC_MAX, -BEAM_MID_M, BEAM_MID_M);
}

/// Devuelve el ángulo de la barra filtrado con un filtro complementario.
float estimateAngle(float prev_ang) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  ang_vel = (g.gyro.x - gyro_bias) * 180.0 / PI;
  float ang_gyro = prev_ang + DT * (g.gyro.x - gyro_bias) * 180.0 / PI;
  float ang_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI - acc_bias;

  return ALPHA * ang_acc + (1 - ALPHA) * ang_gyro;
}

float readSonar() {
  unsigned int us = sonar.ping();
  if (us == 0) {
    return (float)SONAR_MAX_CM / 100.0;
  }
  return (float)us / SONAR_US_PER_M;
}

void matlab_send(float data[SEND_COUNT]){
  // Envío de header
  Serial.write("abcd");
  
  for (int i = 0; i < SEND_COUNT; i++) {
    byte * b = (byte *) &data[i];
    Serial.write(b, sizeof(float));
  }
}

float remap(float x, float from_min, float from_max, float to_min, float to_max) {
  return (x - from_min) / (from_max - from_min) * (to_max - to_min) + to_min;
}

float saturate(float x, float min, float max) {
  if (x > max) {
    x = max;
  } else if (x < min) {
    x = min;
  }
  return x;
}

void runPeriodicallyMicros(unsigned long period) {
  unsigned long curr_time = micros();
  unsigned long elapsed = curr_time - last_time;
  long sleep_time = (long)(period - elapsed);
  last_time += period;
  
  if (sleep_time > 0) {
    delayMicrosecondsFixed(sleep_time);
  }
}

void delayMicrosecondsFixed(unsigned long us) {
  while (us > MAX_DELAY_US) {
    us -= MAX_DELAY_US;
    delayMicroseconds(MAX_DELAY_US);
  }
  delayMicroseconds(us);
}
