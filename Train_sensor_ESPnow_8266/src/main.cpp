/****************************************************
 *  TITLE: ESP-NOW Controller + Ultrasonic sensor
 *  source: https://youtu.be/_cNAsTB5JpM
 *  Board Settings: ESP8266 + US sensor
 *  Board: ESP8266
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

const int sensor_name = 1;  // 0 for left, 1 for right
const int TRIGGER_PIN = 16;   //D0 Or GPIO-16 of nodemcu
const int ECHO_PIN = 5;    //D1 Or GPIO-5 of nodemcu
const int led_2nd = 2;  //D4 Or GPIO-2 of nodemcu
const int MAX_DIST=80;

//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DIST);

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

//int distance;
uint8_t receiverAddress[] = {0x50, 0x02, 0x91, 0x69, 0x6F, 0x22};   //  MAC address of the receiver

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
  //long duration;
  int l_distance;
  
  l_distance = ultrasonic.read();
  //l_distance = sonar.ping_cm();

//Serial.print("  ....  ");
//Serial.print(l_distance);
//Serial.print("     ....  ");
delay(300);
  return l_distance;
}

void setup() {
  pinMode(2, OUTPUT);     // GPIO2 (D4) led on ESP8266 module (in addition to onboard led on GPIO 16)
  // on board led is GPIO16 (D0)
  pinMode(TRIGGER_PIN, OUTPUT);  // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);   // Sets the echoPin as an Input

  Serial.begin(9600);     // initialize serial port
  
  Serial.println();
  Serial.print("Initializing...");
  Serial.print(MY_NAME);
  Serial.print("    My MAC address is: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();        // we do not want to connect to a WiFi network

  if(esp_now_init() != 0) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_set_self_role(MY_ROLE);   
  esp_now_register_send_cb(transmissionComplete);   // this function will get called once all data is sent
  esp_now_add_peer(receiverAddress, RECEIVER_ROLE, WIFI_CHANNEL, NULL, 0);

  Serial.println("Initialized.");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  int distance = measure_dist(TRIGGER_PIN,ECHO_PIN);
  dataPacket packet;
  
  packet.sensor_name = sensor_name;
  packet.sensor_dist = distance;

  Serial.print("Sensor: ");
  Serial.print(sensor_name);
  Serial.print("    distance:  ");
  Serial.print(distance);
  Serial.print(" ");

  
  esp_now_send(receiverAddress, (uint8_t *) &packet, sizeof(packet));
  
  
  digitalWrite(LED_BUILTIN, LOW);
  delay(400);
}