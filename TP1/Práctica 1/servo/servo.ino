#include <Servo.h>
#define PIN_SERVO 9

#define PIN_POTE A3
#define ADC_MAX 1023
#define DELAY_US_50HZ 20000
#define DELAY_US_10HZ 100000
#define DELAY_US_1HZ 1000000
#define DELAY DELAY_US_10HZ

#define WAIT_MS 1500
#define MAX_DELAY_US 16000

int times[] = { 600, 1500, 2400 };
const int LEN_TIMES = sizeof(times) / sizeof(times[0]);

int angles[] = { 0, 90, 180 };
const int LEN_ANGLES = sizeof(angles) / sizeof(angles[0]);

Servo miservo;

void setup() {
  Serial.begin(115200);
  miservo.attach(PIN_SERVO);
}

int i = 0;

void loop1() {
  miservo.writeMicroseconds(times[i++]);
  // miservo.write(angles[i++]);
  if (i >= LEN_TIMES) i = 0;
  // if (i >= LEN_ANGLES) i = 0;
  delay(WAIT_MS);
}

void loop() {
  unsigned long tiempoInicio = micros();

  int value = analogRead(PIN_POTE);
  float ang = value*180.0f/ADC_MAX;
  miservo.write(ang);
  Serial.println(ang);

  unsigned long tiempoFinal = micros();
  unsigned long duracion = tiempoFinal - tiempoInicio;
  unsigned long sleep_time = DELAY-duracion;

  while (sleep_time > MAX_DELAY_US) {
    sleep_time -= MAX_DELAY_US;
    delayMicroseconds(MAX_DELAY_US);
  }

  delayMicroseconds(sleep_time);
}