#include<ESP8266WiFi.h>
#include<espnow.h>

#include<classes.cpp>
//#include<headers.h>  

//#define MOTOR_PWM 10
//#define MOTOR_EN1 9
//#define MOTOR_EN2 8
#define RED_LED   4
#define GREEN_LED 3
#define BLUE_LED  2
#define LED_CMD 2
#define LEDS_OFF_CMD 4
#define RED   1
#define GREEN 2
#define BLUE  3


//const int ZERO = 0;
const int Num_Of_Slaves = 5;
const int Addr_Space = 127;
const int left_dev = 9;
const int right_dev = 10;
//const int LEFT = 0;
//const int RIGHT = 1;
const int UNKNOWN = 2;
const int STOP = 0;
const int ON_MOVE = 1;
// const int VERY_CLOSE = 20;
//const int CLOSE = 20;
//const int IN_RANGE = 40;
//const int MIN_SPEED = 120; // was 120
const int SLOW_SPEED = 140;
//const int MAX_SPEED = 180; // was 180
//const int SPEED_INC = 5;
//const int TIME_IN_STATION = 5000;
//const int SAMPLE_TIME = 200;
const int SPEEDING = 1;
const int SLOWING = 2;
const int STOPPING = 3;

unsigned long curr_time, time_prev;
const int DIST_ARRAY_SIZE = 100;
int dist_cnt = 0;

Sensor left_sensor;
Sensor right_sensor;

struct __attribute__((packed)) dataPacket {
  int sensor_name;
  int sensor_dist;
};
dataPacket packet;

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&packet, incomingData, sizeof(packet));

  if (packet.sensor_name==LEFT) {
    left_sensor.distance_read = packet.sensor_dist;
  }
  else {
    right_sensor.distance_read = packet.sensor_dist;
  }
  Serial.print("Received from sensor: ");
  Serial.print(packet.sensor_name);
  Serial.print("  distance: ");
  Serial.println(packet.sensor_dist);  
}  // of OnDataRecv function


Motor train_motor;

int last_good_dist=IN_RANGE;


void slow_down();
void increase_speed();
void decrease_speed();

// ******************* SETUP ***************/
void setup()
{
  Serial.begin(9600);
  Serial.println("Starting Setup");
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, register it for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(RED_LED  , OUTPUT);
  pinMode(BLUE_LED , OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  
  
  digitalWrite(RED_LED , LOW);
  digitalWrite(BLUE_LED , LOW);
  digitalWrite(GREEN_LED, LOW);

  train_motor.stop(); // make sure it is stopped
  
  Serial.println("******** End of SETUP ****");
}




// ******************* LOOP ***************/
// ****************************************/

void loop()
{

  if (RIGHT == train_motor.direction)
    {
      train_motor.distance = right_sensor.distance_read;
      //Serial.print("R>> ");
    }
  else
    {
      train_motor.distance = left_sensor.distance_read;
      //Serial.print("L>> ");
    }

  train_motor.move_on();  // decides what to do next (stop/keep going / change direction)

  if (DEBUG_MODE) {
    Serial.print("dir: ");
    if (train_motor.direction==LEFT)
      Serial.print("LEFT ");
    else
      Serial.print("RIGHT");
  
    Serial.print(" Speed: ");
    Serial.print(train_motor.speed);
    Serial.print("   Distance: ");
    Serial.println(train_motor.distance);
  }
  delay(SAMPLE_TIME);

}

// *****************************************
// **************** END LOOP ***************
// *****************************************
