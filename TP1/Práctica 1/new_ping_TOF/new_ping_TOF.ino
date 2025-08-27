#include <NewPing.h>

#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define DELAY_50HZ 20000
#define WAIT_MS    50

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup de pings y distancia máxima.

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop1() {
  delay(WAIT_MS);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned long tiempoInicio = micros();  // Marca de tiempo de inicio

  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).

  unsigned long tiempoFinal = micros();                 // Marca de tiempo de finalización
  unsigned long duracion = tiempoFinal - tiempoInicio;  // Calcula la duración

    Serial.print("El código tardó: ");
    Serial.print(duracion);
    Serial.println(" us");
    Serial.println((float)uS / US_ROUNDTRIP_CM);
}


void loop() {
  unsigned long tiempoInicio = micros();  // Marca de tiempo de inicio

  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).

  Serial.print("La distancia es de: ");
  // Serial.print((float)uS / US_ROUNDTRIP_CM);
  Serial.print((float)uS / US_ROUNDTRIP_CM);
  Serial.println(" cm");
      
  unsigned long tiempoFinal = micros();                 // Marca de tiempo de finalización
  unsigned long duracion = tiempoFinal - tiempoInicio;  // Calcula la duración
  delayMicroseconds(DELAY_50HZ-duracion);
}