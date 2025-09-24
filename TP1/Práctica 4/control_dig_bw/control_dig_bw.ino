#include <Adafruit_MPU6050.h>
#include <NewPing.h>
#include <Servo.h>

#define PI 3.14159265359

#define MAX_DELAY_US 16383

#define ADC_BITS 10
#define ADC_MAX  ((2 << ADC_BITS) - 1)

#define BAUD_RATE  115200
#define SEND_COUNT 3

#define CONTROL_PERIOD_US  20000 // 20 ms (50 Hz)
#define DT                 (CONTROL_PERIOD_US / 1e6)

#define BEAM_MID_M 0.2 // Mitad de la longitud de la barra en metros (MEDIR)
#define CART_MID_M 0.028  // Mitad de la longitud del carro (MEDIR)

#define CALIB_ITERS 25 // Medio segundo de calibración

#define SERVO_AMP_DEG 90
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

#define SONAR_MAX_CM   50
#define SONAR_US_PER_M 5831

#define PIN_POT   A3
#define PIN_SERVO 9
#define PIN_TRIG  6
#define PIN_ECHO  7

#define ALPHA 0.01

#define CALIBRATE 1

unsigned long last_time;

Servo servo;
// NewPing sonar(PIN_TRIG, PIN_ECHO, SONAR_MAX_CM);
Adafruit_MPU6050 mpu;

float gyro_bias;
float acc_bias;

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
  mpu.getEvent(&a, &g, &temp);
  gyro_bias = g.gyro.x;

  acc_bias = 0;
  for (int i = 0; i < CALIB_ITERS; i++){
    delayMicroseconds(CONTROL_PERIOD_US);
    mpu.getEvent(&a, &g, &temp);
    acc_bias += atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  }
  acc_bias /= CALIB_ITERS;
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
  static float un1 = 0.0;
  static float un2 = 0.0;
  static float en1 = 0.0;
  static float en2 = 0.0;
  static float ref = 0;
  static float ang = 0;

  ang = estimateAngle(ang);
  float e = ref - ang;
  float u =2.1549*e - 1.8356*en1 + un1;
  //float u = 4.0/3 * un1 - 1.0/3 * un2 + 18.06*e - 29.55*en1 + 12.09*en2;
  // float u = 4.0/3 * un1 - 1.0/3 * un2 + 18.15*e - 29.7*en1 + 12.15*en2;
  un2 = un1;
  un1 = u;
  en2 = en1;
  en1 = e;

  moveServo(u);

  float datos[] = { ang, u, e};
  //matlab_send(datos);

  // Serial.print(ref);
  // Serial.print(", ");
  // Serial.print(pos);
  // Serial.print(", ");
  Serial.print(ang);
  Serial.println();
}

/// Toma valores entre -90 y 90 grados.
/// Satura valores por fuera de ese rango.
void moveServo(float angle) {
  angle = saturate(angle, -SERVO_AMP_DEG, SERVO_AMP_DEG);
  unsigned int us = remap(angle, -SERVO_AMP_DEG, SERVO_AMP_DEG, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(us);
}

/// Devuelve la posición del centro del carrito
/// respecto del centro de la barra, en metros.

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
  
  float ang_gyro = prev_ang + DT * (g.gyro.x - gyro_bias) * 180.0 / PI;
  float ang_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI - acc_bias;

  return ALPHA * ang_acc + (1 - ALPHA) * ang_gyro;
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
