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
const int VERY_CLOSE = 20;
const int CLOSE = 20;
const int IN_RANGE = 40;
const int MIN_SPEED = 120; // was 120
const int SLOW_SPEED = 140;
const int MAX_SPEED = 180; // was 180
const int SPEED_INC = 5;
const int TIME_IN_STATION = 5000;
const int SAMPLE_TIME = 200;
const int SPEEDING = 1;
const int SLOWING = 2;
const int STOPPING = 3;
// const int JUNK_VAL = 7777;
// const int DIST_BUFF_SIZE = 20;

 // int train_speed, L_distance, R_distance, train_direction, train_status;
//int current_dev, current_dist;
unsigned long curr_time, time_prev;

const int DIST_ARRAY_SIZE = 100;

int time_array[DIST_ARRAY_SIZE];
int speed_array[DIST_ARRAY_SIZE];
int dist_cnt = 0;


/*
class Sensor {
  public:
    int distance_read;
    void Turn_led_ON ( int _side, int _led) {
      // transmit to LED on the slected side 
      } // of Turn_led_ON
    
    void Turn_leds_OFF ( int _side) {
      // was transmit to selected device about leds 
      } // of Turn_led_OFF
}; // of Slave class
*/


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
  //Serial.print("        MAC of sensor: ");
  //Serial.println(mac);
  
}







/*
class Motor {
  public:
  int train_speed = 0; // in work, to use as the speed of the train instead of global variable
  int direction = LEFT;

    void Go ( int _direction, int _speed) {
      if (LEFT == _direction) {
        Go_left(_speed);
      }
      else {
        Go_right(_speed);
      }
    }

    void Go_left ( int _speed_) {
      digitalWrite(MOTOR_EN1, HIGH);
      digitalWrite(MOTOR_EN2, LOW);
      analogWrite(MOTOR_PWM, _speed_);
    }

    void Go_right ( int _speed_) {
      digitalWrite(MOTOR_EN1, LOW);
      digitalWrite(MOTOR_EN2, HIGH);
      analogWrite(MOTOR_PWM, _speed_);
    }
    void stop () {
      digitalWrite(MOTOR_EN1, LOW);
      digitalWrite(MOTOR_EN2, LOW);
      analogWrite(MOTOR_PWM, ZERO);
    }

};  // of Motor class
*/



Motor train_motor;


// int bad_reads_cnt = 0;
//int good_reads_cnt = 0;
// int dist_buffer[DIST_BUFF_SIZE];
int last_good_dist=IN_RANGE;

int read_distance(int dev);
void slow_down();
void increase_speed();
void decrease_speed();

// ******************* SETUP ***************/
void setup()
{
  // int l0;

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

  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  
  digitalWrite(BLUE_LED, HIGH);
  delay(2000);
  digitalWrite(BLUE_LED, LOW);

/*
  for (int i=0;i<DIST_BUFF_SIZE;i++) {
    dist_buffer[i]=  JUNK_VAL;
  }
*/

  train_motor.stop(); // make sure it is stopped
  //time_prev = millis();

  Serial.begin(9600);
  Serial.println("Starting Setup");
  
  // train_speed = 0;
  //train_motor.speed = 0;

  for (int i = 0; i < DIST_ARRAY_SIZE; i++)
  {
   
    time_array[i] = JUNK_VAL;
    speed_array[i] = JUNK_VAL;
  }
  //train_speed = 0;
  //train_motor.Go(train_motor.direction , train_motor.speed);  // TBD use here the speed inside the class

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



  

  // the train is assumed to be moving here
  // if it is in station, it will be in the fucntion, not here

  boolean stopping=false;
  if (train_motor.distance < VERY_CLOSE )
    {
    train_motor.stop();
    train_motor.speed = ZERO;
    stopping=true;
    }
  else if (train_motor.distance < CLOSE )
    {
      slow_down();  
    }
    else if (train_motor.distance < IN_RANGE )
      {
        decrease_speed();
      }
      else
        {
          increase_speed();
        }
          
  
  if (!stopping)
    train_motor.Go(train_motor.direction, train_motor.speed);
  else {
      // the train is stopping
      // print the last dist readings

    
   //  delay(5000); // wait 10 seconds 
      // change directions
      if (train_motor.direction == LEFT)
        {
        train_motor.direction = RIGHT;
//        slave.Turn_led_ON (right_dev, BLUE);
//        slave.Turn_leds_OFF(left_dev);
        } // of if
      else
        {
        train_motor.direction = LEFT;
//        slave.Turn_led_ON (left_dev, BLUE);
//        slave.Turn_leds_OFF(right_dev);
        } // of else
    } // of else

/*
  Serial.print("msg1: dist: ");
  Serial.print(current_dist);
  Serial.print(" , speed: ");
  Serial.print(train_speed);
  Serial.print(" good reads: ");
  Serial.print(good_reads_cnt);
  Serial.print(" bads: ");
  Serial.println(bad_reads_cnt);
  */

  delay(SAMPLE_TIME);
  // delay(500); // for DEBUG


}

// *****************************************
// **************** END LOOP ***************
// *****************************************

// ****************** SLOW_DOWN **********************
void slow_down() {
  train_motor.speed = MIN_SPEED;
}
// ****************** increase_speed **********************
void increase_speed() {
  train_motor.speed += SPEED_INC;
  if (train_motor.speed > MAX_SPEED)
    train_motor.speed = MAX_SPEED;
}
// ****************** decrease_speed **********************
void decrease_speed() {
  train_motor.speed -= SPEED_INC;
  if (train_motor.speed < MIN_SPEED)
    train_motor.speed = MIN_SPEED;
}

// ****************** read_distance **********************
int read_distance(int dev)
{
  int dist=777; // TBD
  return dist;
}

