// AS5600 Magnetic Encoder Test

#define pot_pin A0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pot_val = analogRead(pot_pin);
  Serial.println(pot_val);
  
  
}
