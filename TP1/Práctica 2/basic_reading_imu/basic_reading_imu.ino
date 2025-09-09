// Basic demo for accelerometer readings from Adafruit MPU6050
#define MAX_DELAY_US 16000
#define DELAY_US_50HZ 20000
#define DELAY_US_10HZ 100000
#define DELAY_US_1HZ 1000000
#define DELAY DELAY_US_50HZ


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  unsigned long tiempoInicio = micros();

  float data[6];
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  unsigned long tiempoInicioACQ = micros();
  mpu.getEvent(&a, &g, &temp);
  unsigned long tiempoFinalACQ = micros();

  data[0] = a.acceleration.x;
  data[1] = a.acceleration.y;
  data[2] = a.acceleration.z;
  data[3] = g.gyro.x;
  data[4] = g.gyro.y;
  data[5] = g.gyro.z;
  
  matlab_send(data, 6);
  
  unsigned long tiempoFinal = micros();
  unsigned long duracion = tiempoFinal - tiempoInicio;
  unsigned long sleep_time = DELAY-duracion;
  
  unsigned long duracionACQ = tiempoFinalACQ - tiempoInicioACQ;
  Serial.println(duracionACQ);

  while (sleep_time > MAX_DELAY_US) {
    sleep_time -= MAX_DELAY_US;
    delayMicroseconds(MAX_DELAY_US);
  }

  delayMicroseconds(sleep_time);
}


void matlab_send(float * data, unsigned int length){
  // Envío de header
  Serial.write("abcd");
  
  for (int i = 0; i < length; i++) {
    byte * b = (byte *) &data[i];
    Serial.write(b, sizeof(float));
  }
}

