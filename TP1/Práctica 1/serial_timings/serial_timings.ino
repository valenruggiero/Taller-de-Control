#define NUM_FLOATS 100

float data[NUM_FLOATS] = {0};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_FLOATS; ++i) {
    data[i] = (float)i;
  }
}

void loop() {
  unsigned tick = micros();
  for (size_t i = 0; i < NUM_FLOATS; i++)
    Serial.write((uint8_t*)&data[i], sizeof(data[0]));
  unsigned tock = micros();
  Serial.println();
  Serial.println((float)(tock - tick) / NUM_FLOATS);
  delay(1000); 
}
