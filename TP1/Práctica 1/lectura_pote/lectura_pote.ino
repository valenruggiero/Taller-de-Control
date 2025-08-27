#define PIN_POTE A3
#define ADC_MAX 1023
#define DELAY_50HZ 20000

void setup()  {
  Serial.begin(9600); // Inicia comunicación serial
  // Código que no se medirá
}

void loop1() {
  unsigned long tiempoInicio = micros();  // Marca de tiempo de inicio

  // inicializo valor
  int value = analogRead(PIN_POTE);

  unsigned long tiempoFinal = micros();                 // Marca de tiempo de finalización
  unsigned long duracion = tiempoFinal - tiempoInicio;  // Calcula la duración

    Serial.print("El código tardó: ");
    Serial.print(duracion);
    Serial.println(" us");
}

int last_time = 0;

void loop() {
  unsigned long tiempoInicio = micros();  // Marca de tiempo de inicio

  // inicializo valor
  int value = analogRead(PIN_POTE);
  float ang_dec = (float)value/ADC_MAX * 180.0f - 90 ;


  //Serial.print("El ángulo en decenas de grados es: ");
  Serial.print(ang_dec);
  Serial.print(",");
  Serial.println(last_time);
  //Serial.println("");

  unsigned long tiempoFinal = micros();             // Marca de tiempo de finalización
  unsigned long duracion = tiempoFinal - tiempoInicio;  // Calcula la duración
  last_time = duracion;

  delayMicroseconds(DELAY_50HZ-duracion);

}
