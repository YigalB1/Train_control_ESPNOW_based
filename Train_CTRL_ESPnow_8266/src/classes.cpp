#include<headers.h>
#include<ESP8266WiFi.h>

class Motor {
  public:
  int train_speed = 0; // in work, to use as the speed of the train instead of global variable
  int direction = LEFT;
  int distance = 0;

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
