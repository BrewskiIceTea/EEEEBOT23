#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <stdlib.h>
#include <stdio.h>

const int trigPin = 5;  
const int echoPin = 18  ; 

float duration, distance;  

String message;
float encoderDist;

const char* ssid = "C01-08";       
const char* password = "87654321";                      
const char* mqtt_server = "192.168.137.62";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

const int ledPin = 4;

void setup() {
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);  
  
	Serial.begin(9600);
  Serial.print("STARTING");  

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();             // join i2c bus with address 8
  delay(2000);  
}

void setup_wifi() {
  delay(10);
  // we start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // feel free to add more if statements to control more GPIOs with MQTT

  // if a message is received on the topic esp32/wiggle, you check if the message is either "on" or "off". 
  // changes the output state according to the message
  if (String(topic) == "esp32/wiggle") {
    Serial.println("Wiggling");
      Wire.beginTransmission(0x08); // transmit to slave device address 8
      Wire.write("W");
      Wire.endTransmission();       // end transmission

  }
}
void reconnect() {
  // loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // subscribe
      client.subscribe("esp32/wiggle");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void loop() {
  if (!client.connected()) { // Reconnecting client if connection lost
    reconnect();
  }
  client.loop();
  delay(100);
  // requestEncoders();

  //HC-SR04 Write
  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);  

  //HC-SR04 Read
  sensorRead();
  requestEncoders();
  // stopCheck();
	delay(100);  
}

void stopCheck(){
  if (distance <=10){
    Wire.beginTransmission(0x08); // transmit to slave device address 8
    Wire.write("S");
    Wire.endTransmission();       // end transmission
    Serial.println("Sending STOP request");
  }
}

void sensorRead(){
  duration = pulseIn(echoPin, HIGH);  
  distance = (duration*.0343)/2;  
  Serial.print("Distance: ");
	Serial.println(distance);
  String distanceStr = String(distance);

  // Convert String to char*
  const char* distanceChar = distanceStr.c_str();

  // Publish the distance
  client.publish("esp32/sensorDist", distanceChar);
}

void requestEncoders(){
  Wire.requestFrom(0x08, 7);
  String receivedMessage;
  if (Wire.available()){
    Serial.println("\nMessage Recieved");
  }
  while (Wire.available()) {
    char c = Wire.read();
    receivedMessage += c;
  }

  if (receivedMessage.length() == 7 && receivedMessage != message) {
    message = receivedMessage;
    Serial.println("\nMessage Received");
    Serial.println(message);  // Write the complete message to the serial monitor

    encoderDist = message.toFloat();

    // Convert float to string
    String encoderDistStr = String(encoderDist);
    client.publish("esp32/encoderDist", encoderDistStr.c_str());
  }
  delay(100);
}


