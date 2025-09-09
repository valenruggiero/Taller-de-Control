// Basic demo for accelerometer readings from Adafruit MPU6050
#define MAX_DELAY_US 16000
#define DELAY_US_50HZ 20000
#define DELAY_US_10HZ 100000
#define DELAY_US_1HZ 1000000
#define DELAY DELAY_US_50HZ
#define DELTA_T 20e-3
#define N_ITER 10
#define PI 3.14159265359


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float theta_g = 0;
float g_bias;
float a_bias = 0;
float alpha = 0.2; // ajustable
float theta_c = 0;

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

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  g_bias = g.gyro.x;
  for (int i = 0; i<N_ITER; i++){
    delay(50);
    mpu.getEvent(&a, &g, &temp);
    a_bias += atan2(a.acceleration.y, a.acceleration.z)*180/PI;
  }

  a_bias /= N_ITER;

}

void loop() {
  unsigned long tiempoInicio = micros();

  float data[3];
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  theta_g = theta_g + DELTA_T * (g.gyro.x - g_bias) * 180/PI; // diverge
  float theta_g_exact =  theta_c + DELTA_T * (g.gyro.x - g_bias) * 180/PI; // corregido
  float theta_a = atan2(a.acceleration.y, a.acceleration.z)*180/PI - a_bias; // ruidoso
  //atan2(a.acceleration.y - ay_bias, a.acceleration.z - az_bias)*180/PI;

  // FILTRO COMPLEMENTARIO
  theta_c = theta_a*alpha + theta_g_exact*(1-alpha);


  data[0] = theta_g;
  data[1] = theta_a;
  data[2] = theta_c;
  
  matlab_send(data, 3);
  
  unsigned long tiempoFinal = micros();
  unsigned long duracion = tiempoFinal - tiempoInicio;
  unsigned long sleep_time = DELAY-duracion;
  

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

