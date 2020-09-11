#include<headers.h>
#include<ESP8266WiFi.h>

class Motor {
  public:
  int speed = 0; // in work, to use as the speed of the train instead of global variable
  int direction = LEFT;
  int distance = 0;
  int bad_reads_cnt = 0;
  int bad_reads_in_a_row = 0;
  int good_reads_cnt = 0;
  int dist_buffer[DIST_BUFF_SIZE];

  void init() {
    for (int i=0;i<DIST_BUFF_SIZE;i++) {
      dist_buffer[i]=  JUNK_VAL;
    }
  } // of INIT routine

    void Go ( int _direction, int _speed) {
      if (LEFT == _direction) {
        Go_left(_speed);
      }
      else {
        Go_right(_speed);
      }
    } // of GO routine


    void Go_left ( int _speed_) {
      digitalWrite(MOTOR_DIR, LEFT);      
      analogWrite(MOTOR_PWM, _speed_);
    } // of GO LEFT routine


    void Go_right ( int _speed_) {
      digitalWrite(MOTOR_DIR, RIGHT);         
      analogWrite(MOTOR_PWM, _speed_);
    } // of GO RIGHT routine


    void stop () {
      analogWrite(MOTOR_PWM, ZERO);
    } // of STOP routine


    // ****************** increase_speed **********************
    void increase_speed() {

      if (fixed_speed) {
        speed = MAX_SPEED;
        return; // speed is not changing
      }
        
      // speed is not fixed

      speed += SPEED_INC;
      if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    }
    // ****************** decrease_speed **********************
    void decrease_speed() {
      if (fixed_speed)
        return; // speed is not changing

      speed -= SPEED_INC;
      if (speed < MIN_SPEED)
        speed = MIN_SPEED;
    }

// ****************** SLOW_DOWN **********************
void slow_down() {
  if (fixed_speed)
    return; // speed is not changing

  speed = MIN_SPEED;
} // of SLOW DOWN








    void calc_speed() {
      if (distance < 2)
      {
        bad_reads_cnt += 1;
        bad_reads_in_a_row +=1;
        return; // bad read
      }
      // read is good
      bad_reads_in_a_row = 0;
      good_reads_cnt += 1;
      if (distance > 100) {
        distance=101;
      }
      for (int i=DIST_BUFF_SIZE-1;i>=1;i--) {
        dist_buffer[i] = dist_buffer[i-1];
      }
      dist_buffer[0] = distance;   
    } // of CALC SPEED routine

    void move_on() {
      // here we actually move the train, or stop it in the stattion
      if (distance < VERY_CLOSE )
      {
        stop(); // change output pins to stop the train
        speed = ZERO;
        delay(TIME_IN_STATION);

        if (direction == LEFT) {
          direction = RIGHT;
        }
        else {
          direction = LEFT;
        }
        return;
      } // of IF 

      if (distance < CLOSE ) {
        slow_down(); 
        Go(direction, speed);
        return;
      }

    if (distance < IN_RANGE )
      {
        decrease_speed();
        Go(direction, speed);
        return;
      }
      
    increase_speed();
    Go(direction, speed);
    return;    
          
  
  






    } // of move_on

};  // of Motor class



class Sensor {
  public:
    int distance_read;
    void Turn_led_ON ( int _side, int _led) {
      // transmit to LED on the slected side 
      } // of Turn_led_ON
    
    void Turn_leds_OFF ( int _side) {
      // was transmit to selected device about leds 
      } // of Turn_led_OFF
}; // of Sensor class
