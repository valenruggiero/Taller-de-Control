#define SEND_COUNT 1

float values[100];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 100; ++i) {
    values[i] = (float)rand()/RAND_MAX;
  }
}

void loop() {
  unsigned long tick = micros();
  for (int i = 0; i < 100; ++i) {
    matlab_send(&values[i]);
  }
  unsigned long tock = micros();
  Serial.print("\nTook: ");
  Serial.print(tock - tick);
  Serial.println(" us");

  while(1) delay(10);
}

void matlab_send(float data[SEND_COUNT]){
  // Envío de header
  Serial.write("abcd");
  
  for (int i = 0; i < SEND_COUNT; i++) {
    byte * b = (byte *) &data[i];
    Serial.write(b, sizeof(float));
  }
}