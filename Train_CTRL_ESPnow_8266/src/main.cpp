// ESP8266 with motor shield
// info here: https://hackaday.io/project/8856-incubator-controller/log/29291-node-mcu-motor-shield
// and here: https://cdn.hackaday.io/files/8856378895104/user-mannual-for-esp-12e-motor-shield.pdf

#include<ESP8266WiFi.h>
#include<espnow.h>
#include<classes.cpp>

Sensor left_sensor;
Sensor right_sensor;
bool led_on = true;

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
  /*
  Serial.print("Received from sensor: ");
  Serial.print(packet.sensor_name);
  Serial.print("  distance: ");
  Serial.println(packet.sensor_dist);  
  */
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
  Serial.print("");
  Serial.print("Starting Setup      ");
   Serial.print("ESP8266 Motor driver Board MAC Address:  ");
  Serial.print(WiFi.macAddress());

  pinMode(MOTOR_DIR   , OUTPUT);
  pinMode(MOTOR_PWM   , OUTPUT);
  pinMode(ON_BOARD_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  // start with LEDs on, to make sure it works well
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED,HIGH);
  digitalWrite(RED_LED,   HIGH);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("    Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, register it for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);


  train_motor.stop(); // make sure it is stopped
  
  Serial.println("       End of SETUP ****");
}




// ******************* LOOP ***************/
// ****************************************/

void loop()
{
  if (led_on) {
  digitalWrite(ON_BOARD_LED, HIGH);
  led_on=false;
  }
  else {
    digitalWrite(ON_BOARD_LED, LOW);
    led_on=true;
  }
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

  if (777 == train_motor.distance) {
    // 777 means no reading 
    train_motor.distance = 0; // make the train stop, since no reading 
  }



  train_motor.move_on();  // decides what to do next (stop/keep going / change direction)
  


  if (DEBUG_MODE) {
    String msg;
    msg.concat("dir: ");
    //Serial.print("dir: ");
    if (train_motor.direction==LEFT) {
      msg.concat("LEFT  ");
      //Serial.print("LEFT ");
    }
    else{
      msg.concat("RIGHT ");
      //Serial.print("RIGHT");
    }
      
    msg.concat("Speed: ");
    msg.concat(train_motor.speed);
    msg.concat("  Distance:");
    msg.concat(train_motor.distance);
    msg.concat("   LEFT  Distance: ");
    if (left_sensor.distance_read==777) 
      msg.concat("..");
    else
      msg.concat(left_sensor.distance_read);
    msg.concat("   RIGHT Distance: ");
    if (right_sensor.distance_read==777) 
      msg.concat("..");
    else
      msg.concat(right_sensor.distance_read);
    Serial.println(msg);
  } // of IF debug
  //delay(SAMPLE_TIME);

}
// *****************************************
// **************** END LOOP ***************
// *****************************************