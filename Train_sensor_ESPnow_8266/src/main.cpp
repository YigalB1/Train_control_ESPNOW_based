/****************************************************
 *  TITLE: ESP-NOW Controller + Ultrasonic sensor
 *  source: https://youtu.be/_cNAsTB5JpM
 *  Board Settings: ESP8266 + US sensor
 *  Board: ESP8266
 * Two ultrasonic sensor (named 0 - Left,1 - Right) with ESP32 reading distance of train from the stattion
 * Sending it to ESp8266 with motor driver
 ****************************************************
 */

#include<ESP8266WiFi.h>
#include<espnow.h>
#include <Ultrasonic.h>
//#include <NewPing.h>

#define MY_NAME         "CONTROLLER_NODE"
#define MY_ROLE         ESP_NOW_ROLE_CONTROLLER         // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_SLAVE              // set the role of the receiver
#define WIFI_CHANNEL    1

#define LEFT  0
#define RIGHT 1

const int sensor_name = RIGHT;  // 0 for Left, 1 for Right
const int TRIGGER_PIN = 5; // was 16;   //D0 Or GPIO-16 of nodemcu
const int ECHO_PIN = 4; // was 5;    //D1 Or GPIO-5 of nodemcu
const int RED_LED_PIN = 12; 
const int YELLOW_LED_PIN = 13; 
const int GREEN_LED_PIN = 15; 
const int MAX_DIST=80;
const int ON_BOARD_LED = 2; //D4 Or GPIO-2 of nodemcu
const int DELAY_CYCLE = 100 ; // time to wait before next measurement
const int IN_RANGE = 50; // Sensor's RED led will be ON when in range



bool LED_ON = true ;

void test_hw();

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

// uint8_t receiverAddress[] = {0x50, 0x02, 0x91, 0x69, 0x6F, 0x22};   //  MAC address of the receiver Damaged :(
// uint8_t receiverAddress[] = {0xEC, 0xFA, 0xBC, 0xC3, 0x01, 0xA0};   //  MAC address of the receiver
uint8_t receiverAddress[] = {0x5C, 0xCF, 0x7F, 0x40, 0xE1, 0x26};   //  MAC address of the receiver

struct __attribute__((packed)) dataPacket {
  int sensor_name;
  int sensor_dist;
};

void transmissionComplete(uint8_t *receiver_mac, uint8_t transmissionStatus) {
  if(transmissionStatus == 0) {
    Serial.println(" ... Data sent successfully");
  } else {
    Serial.print(" .. Error code: ");
    Serial.println(transmissionStatus);
  }
}

int measure_dist(int _trig,int _echo)
{
  int l_distance;  
  l_distance = ultrasonic.read();
  return l_distance;
}

// ******************** SETUP ***************************
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // GPIO2 (D4) led on ESP8266 module (in addition to onboard led on GPIO 16)
  // on board led is GPIO16 (D0)
  pinMode(TRIGGER_PIN, OUTPUT);  // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);   // Sets the echoPin as an Input

  pinMode(RED_LED_PIN, OUTPUT);   // Sets the echoPin as an Input
  pinMode(YELLOW_LED_PIN, OUTPUT);   // Sets the echoPin as an Input
  pinMode(GREEN_LED_PIN, OUTPUT);   // Sets the echoPin as an Input


  

  Serial.begin(9600);     // initialize serial port
  
  Serial.println();
  Serial.print("Initializing...");
  Serial.print(MY_NAME);
  Serial.print("    My MAC address is: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();        // Just in case, we do not want to connect to a WiFi network

  if(esp_now_init() != 0) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_set_self_role(MY_ROLE);   
  esp_now_register_send_cb(transmissionComplete);   // this function will get called once all data is sent
  esp_now_add_peer(receiverAddress, RECEIVER_ROLE, WIFI_CHANNEL, NULL, 0);

  Serial.println("Initialized.");
  test_hw();
  Serial.println("Les'd tested");

  // Turn on GREEN led for sensor 0, or YELLOW for sensor 1, contantly
  // RED led will signal in range (in loop)
  if  (sensor_name == LEFT) 
    digitalWrite(GREEN_LED_PIN,HIGH);
  else
    digitalWrite(YELLOW_LED_PIN,HIGH);

}

// ******************** LOOP ***************************

void loop() {
  if (LED_ON) {
    digitalWrite(LED_BUILTIN, HIGH);
    LED_ON=false;
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
    LED_ON=true;
  }
  
  int distance = measure_dist(TRIGGER_PIN,ECHO_PIN);

  if (distance<=IN_RANGE)
    digitalWrite(RED_LED_PIN,HIGH);
  else
    digitalWrite(RED_LED_PIN,LOW);

  dataPacket packet;
  packet.sensor_name = sensor_name;
  packet.sensor_dist = distance;

  Serial.print("Sensor: ");
  Serial.print(sensor_name);
  Serial.print("    distance:  ");
  Serial.print(distance);
  Serial.print(" ");

  
  esp_now_send(receiverAddress, (uint8_t *) &packet, sizeof(packet));
  
  
  delay(DELAY_CYCLE);
}

void write_leds(int _value1, int _value2, int _value3) {
  digitalWrite(GREEN_LED_PIN,_value1);
  digitalWrite(YELLOW_LED_PIN,_value2);
  digitalWrite(RED_LED_PIN,_value3);
}
void test_leds() {
  for (int i=1;i<=3;i++) {
    write_leds(LOW,LOW,LOW);
    delay(500);
    write_leds(HIGH,LOW,LOW);
    delay(500);
    write_leds(LOW,HIGH,LOW);
    delay(500);
    write_leds(LOW,LOW,HIGH);
    delay(500);
    write_leds(HIGH,HIGH,HIGH);
    delay(500);
    write_leds(LOW,LOW,LOW);
  }
}
void test_hw(){
  test_leds();
}