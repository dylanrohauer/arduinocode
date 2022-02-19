#include <Wire.h> //This is for i2C

void setup() {
  Serial.begin(115200); //start serial - tip: don't use serial if you don't need it (speed considerations)
  Wire.begin(); //start i2C
  Wire.setClock(800000L); //fast clock
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("main loop");
  read_conf_register();
  delay(500);
}

void read_conf_register(){
  //char conf_setup[7];
  Wire.beginTransmission(0x36); //connect to the sensor
  //Serial.println("transmission begun");
  Wire.write(0x08); //figure 21 - register map: Status: MD ML MH
  //Serial.println("transmission begun");
  Wire.endTransmission(); //end transmission
  Serial.println("transmission sent");
  Wire.requestFrom(0x36, 1); //request from the sensor

  while(Wire.available()>0){
    char c = Wire.read();
    Serial.println(c);
    }
  //conf_setup = Wire.read();
  //Serial.println("Configuration Setup:");
  //Serial.println(conf_setup);
  }
