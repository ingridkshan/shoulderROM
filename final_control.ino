// The below are variables, which can be changed
int led_state = LOW;    // the current state of LED
int button_state;       // the current state of button
int last_button_state;  // the previous state of button

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU9250_asukiaaa.h>

#define BUTTON_PIN 13  // ESP32 pin GIOP16, which connected to button
#define LED_PIN    18  // ESP32 pin GIOP18, which connected to led

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 23
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor(0x68);
double roll = 0.00, pitch = 0.00;       //Roll & Pitch are the angles which rotate by the axis X and y
double goal = 41;
//
//// Replace the next variables with your SSID/Password combination
//const char* ssid = "Mimi's Crib";
//const char* password = "HumanSlaves";
//
//// Add your MQTT Broker IP address, example:
////const char* mqtt_server = "192.168.1.144";
//const char* mqtt_server = "3.123.180.179";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// the number of the LED pin
const int redPin = 16;  // 16 corresponds to GPIO16
const int greenPin = 17; // 17 corresponds to GPIO17
const int yellowPin = 19;
int motor1Pin1 = 15; 
int motor1Pin2 = 32; 

// setting resistance levels
const char* motorResistance = "off";

// Setting Motor PWM properties
const int motorFreq = 30000;
const int pwmChannel = 0;
int motorDutyCycle = 200;

// setting PWM properties
const int freq = 5000;
const int ledChannelRed = 0;
const int ledChannelGreen = 1;
const int ledChannelYellow = 2;
const int resolution = 8;
const int dutyCycle = 255;

void setup() {
  Serial.begin(115200);                // initialize serial
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(LED_PIN, OUTPUT);          // set ESP32 pin to output mode
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  button_state = digitalRead(BUTTON_PIN);
  
  #ifdef _ESP32_HAL_I2C_H_
  // for esp32
  Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
  #else
  Wire.begin();
  #endif
  
  mySensor.setWire(&Wire);
  
  mySensor.beginAccel();
  mySensor.beginMag();
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannelRed, freq, resolution);
  ledcSetup(ledChannelGreen, freq, resolution);
  ledcSetup(ledChannelYellow, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(redPin, ledChannelRed);
  ledcAttachPin(greenPin, ledChannelGreen);
  ledcAttachPin(yellowPin, ledChannelYellow);

//  setup_wifi();
//  client.setServer(mqtt_server, 1883);
//  client.setCallback(callback);
}

void RP_calculate(){
  double x_Buff = mySensor.accelX();
  double y_Buff = mySensor.accelY();
  double z_Buff = mySensor.accelZ();
  roll = atan2(x_Buff , y_Buff) * 57.3;
  if (roll < 0) {
    roll = (-1)*(roll + 90);
  } else {
    roll = 180 - (roll - 90);
  }
  pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
}

//void setup_wifi() {
//  delay(10);
//  // We start by connecting to a WiFi network
//  Serial.println();
//  Serial.print("Connecting to ");
//  Serial.println(ssid);
//
//  WiFi.begin(ssid, password);
//
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//
//  Serial.println("");
//  Serial.println("WiFi connected");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());
//}
//
//void callback(char* topic, byte* message, unsigned int length) {
//  Serial.print("Message arrived on topic: ");
//  Serial.print(topic);
//  Serial.print(". Message: ");
//  String messageTemp;
//  
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)message[i]);
//    messageTemp += (char)message[i];
//  }
//  Serial.println();
//}

  // Feel free to add more if statements to control more GPIOs with MQTT

//void reconnect() {
//  // Loop until we're reconnected
//  while (!client.connected()) {
//    Serial.print("Attempting MQTT connection...");
//    // Attempt to connect
//    if (client.connect("espClient")) {
//      Serial.println("connected");
//    } else {
//      Serial.print("failed, rc=");
//      Serial.print(client.state());
//    }
//  }
//}

void move_forward() {
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  delay(2000); 
}

void stop_motor() {
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(1000);
}

void move_backward() {
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  delay(2000);
}

void loop() {
//  if (!client.connected()) {
//    reconnect();
//  }
//  client.loop();
  
  ledcWrite(ledChannelYellow, 255); 
  mySensor.accelUpdate();
  last_button_state = button_state;      // save the last state
  button_state = digitalRead(BUTTON_PIN); // read new state
  RP_calculate();
  roll = abs(roll);
//  Serial.println(String(roll));

  if (last_button_state == HIGH && button_state == LOW) {
    Serial.println("The button is pressed");

    // toggle state of LED
    led_state = !led_state;

    // control LED arccoding to the toggled state
    digitalWrite(LED_PIN, led_state);
  }

  if (roll > (0.9)*goal && roll < goal && motorResistance!="low") {
    Serial.println("Approaching goal...");
//    ledcWrite(ledChannelGreen, 255); 
//    ledcWrite(ledChannelRed, 0);
    if (motorResistance == "off"){
      move_forward();
      motorResistance = "low";
    } else if (motorResistance == "med" ){
      move_backward();
      motorResistance = "low";
    }
  } else if (roll > (1.15)*goal && roll < (1.25)*goal && motorResistance!="med") {
    Serial.println("Goal reached...");
//    ledcWrite(ledChannelRed, 255);
//    ledcWrite(ledChannelGreen, 0);
    if (motorResistance == "low"){
      move_forward();
      motorResistance = "med";
    } else if (motorResistance == "high" ){
      move_backward();
      motorResistance = "med";
    }
  } else if (roll > (1.3)*goal &&motorResistance!="high") {
    Serial.println("Goal exceeded...");
//    ledcWrite(ledChannelRed, 255);
//    ledcWrite(ledChannelGreen, 0);
    move_forward();
    motorResistance = "high";
  }
    
  if (roll >= goal) {
    ledcWrite(ledChannelGreen, 255); 
    ledcWrite(ledChannelRed, 0);
  } else {
    ledcWrite(ledChannelRed, 255);
    ledcWrite(ledChannelGreen, 0);
  }
  delay(100);

  char angleString[8];
  dtostrf(roll, 1, 2, angleString);
  Serial.print("Angle: ");
  Serial.println(angleString);
  Serial.print("Goal: ");
  Serial.println(goal);
  client.publish("esp32/angle", angleString);
  delay(250);
}
