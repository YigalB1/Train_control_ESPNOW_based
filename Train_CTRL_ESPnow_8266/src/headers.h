#define MOTOR_PWM       5   // PWM pin: between 0 - 1023 analog, or 0 off 1 full speed digital
#define MOTOR_DIR       0   // 0 forward. 1 backward
#define ON_BOARD_LED    2   //D4 Or GPIO-2 of nodemcu ESP8266
#define GREEN_LED       12
#define YELLOW_LED      13
#define RED_LED         15

const int JUNK_VAL = 7777;
const int DIST_BUFF_SIZE = 20;
const int VERY_CLOSE = 20;
const int CLOSE = 20;
const int IN_RANGE = 40;
const int MIN_SPEED = 120; // was 120
const int MAX_SPEED = 1023; // was 180
const int SPEED_INC = 5;
const int TIME_IN_STATION = 3000;
const int SAMPLE_TIME = 2000; // time for loop to wait between each cycle


const int LEFT  = 0;
const int RIGHT = 1;
const int ZERO  = 0;
const int LEFT_DIR  = 1; // it should have been like LEFT & RIGHT, but sto match the  real engine direction.
const int RIGHT_DIR = 0;

const bool fixed_speed = true; // True: speed is fixed or stopped. False: speed changes (slower or faster)
const bool DEBUG_MODE = true; // when true, printing debug statemens