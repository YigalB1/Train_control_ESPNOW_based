#define MOTOR_PWM 10
#define MOTOR_EN1 9
#define MOTOR_EN2 8

const int JUNK_VAL = 7777;
const int DIST_BUFF_SIZE = 20;
const int VERY_CLOSE = 20;
const int CLOSE = 20;
const int IN_RANGE = 40;
const int MIN_SPEED = 120; // was 120
const int MAX_SPEED = 180; // was 180
const int SPEED_INC = 5;
const int TIME_IN_STATION = 5000;
const int SAMPLE_TIME = 200; // time for loop to wait between each cycle


const int LEFT = 0;
const int RIGHT = 1;
const int ZERO = 0;

const bool fixed_speed = true; // True: speed is fixed or stopped. False: speed changes (slower or faster)
const bool DEBUG_MODE = true; // when true, printing debug statemens