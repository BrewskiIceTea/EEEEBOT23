const int trigPin = 5;  
const int echoPin = 18  ; 

float duration, distance;  

#include <Wire.h>     // include Wire library

void setup() {
  Wire.begin();       // join i2c bus (address optional for master)
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);  
	Serial.begin(9600);
  delay(4000);
  Serial.print("STARTING");  

}

void loop() {
  Wire.beginTransmission(0x08); // transmit to slave device address 8


  //HC-SR04 Read and Write
  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);  

  // Calculations
  duration = pulseIn(echoPin, HIGH);  
  distance = (duration*.0343)/2;  
  if (distance <=10){
    Wire.write("S");
    Serial.println("Sending STOP request");
  }
  Serial.print("Distance: ");  
	Serial.println(distance);  
  Wire.endTransmission();       // end transmission
	delay(100);  

}
