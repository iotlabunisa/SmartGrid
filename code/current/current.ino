#define currentpin 13

float current = 0;
float adc_voltage = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  current = 0;

  for(int i=0; i<100; i++){
    float adc = analogRead(currentpin);
    adc_voltage = (adc * (3.3 / 4095.0));
    current = current + ((adc_voltage - 1.5) / 0.100)/100;
    delay(10);
  }

  Serial.print("Voltage Value: ");
  Serial.println(adc_voltage);
  Serial.print("Current Value: ");
  Serial.println(current);
  delay(500);
}
