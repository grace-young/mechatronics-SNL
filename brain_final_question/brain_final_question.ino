#include <Timers.h>

// ------ TAPE SENSOR PINS -------
#define PIN_SENSOR_A 7 
#define PIN_SENSOR_B_1 6 //not working
#define PIN_SENSOR_C 8
#define PIN_SENSOR_D_1 13
#define PIN_SENSOR_E 12
#define PIN_SENSOR_B_2 A0 // NEW
#define PIN_SENSOR_D_2 A5 //Not Working
#define PIN_SENSOR_F A4

// ----- COMMM PINS --------
#define PIN_COMMS_IN_GYRO        2
#define PIN_OUTPUT_MOTOR_CONTROL 10 //changed from 3
#define PIN_OUTPUT_SPEED_CONTROL 9
#define PIN_GAME_START           11 // same

// ----- COMMM TRANSLATIONS --------
#define COMM_MOTOR_NORMAL  20
#define COMM_MOTOR_SLOW    40
#define COMM_MOTOR_FAST    60
#define COMM_ZERO_GYRO     80

#define COMM_MOTOR_COAST_STOP 20
#define COMM_MOTOR_FORWARD 40
#define COMM_MOTOR_BACKWARD 60
#define COMM_MOTOR_LEFT 80
#define COMM_MOTOR_RIGHT 100
#define COMM_MOTOR_SPIN_CC 120
#define COMM_MOTOR_SPIN_CL 140
#define COMM_MOTOR_STOP 160
#define COMM_MOTOR_BACKWARD_JIGGLE 180

// ----- TIME INTERVALS -----
#define TIME_INTERVAL      500
#define JIGGLE_INTERVAL    400
#define TIME_INTERVAL_2    1000
#define TIME_BETWEEN_LINE_TAPE_READS  500
#define TIME_MOVE_RIGHT    5000
#define TIME_TO_PIVOT      250
#define  TIME_TO_SLAM_LEFT  2000

#define TIME_TO_PAUSE      1250


// ---- CONSTANTS ----
static bool startGyroNegative = false;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;

// ---- STATES -----
typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT,
  AT_BACK_OF_BOX, SLAM_LEFT, LOOK_FOR_FIRST_TAPE,
  FOUND_FIRST_TAPE, SHOOT_FROM_FIRST, LOOK_FOR_SECOND_TAPE, 
  FOUND_SECOND_TAPE, SHOOT_FROM_SECOND, DONE
  
} States_t;

States_t state;

void setup() {
  Serial.begin(57600);

  SetupPins();
  makeMotorSpeedNormal();

  // this is for reading the gyro
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
  fake_timer = millis();
  
  state = WAIT;
}

void loop() {
//  makeMotorSpeedFast();
//  makeMotorsMoveLeft();
  CheckGlobalStates();

}

void CheckGlobalStates(void){
  
  Serial.println(state);
  Serial.println(ReadTapeSensor_B_2());
  Serial.println("--");
  switch (state){
    case WAIT:
      RespWait();
      break;
    case START:
      RespStart();
      break;
    case ORIENTATION_STRAIGHT:
      RespOrientationStraight();
      break;
    case DONE:
      RespDone();
      break;
    case SLAM_LEFT:
      RespSlamLeft();
      // need to test here
      break;
    case EXTEND_EYE:
      RespExtendEye();
      break;
    case LOOK_FOR_FIRST_TAPE:
      RespLookForFirstTape();
      break;
    case FOUND_FIRST_TAPE:
      RespFoundFirstTape();
      break;
    case SHOOT_FROM_FIRST:
      RespShootFromFirst();
      break;
    case LOOK_FOR_SECOND_TAPE:
      RespLookForSecondTape();
      break;
    case FOUND_SECOND_TAPE:
      RespFoundSecondTape();
      break;
    default:
      break;
  }
}

void RespWait(){
  makeMotorsCoastStop();
  if(decodeSignalFromComms() == 0){
      startGyroNegative = true;
  }else{
      startGyroNegative = false;
  }
  if(!digitalRead(PIN_GAME_START)){
    state = START;
  }
}

void RespStart(){
  makeMotorSpeedNormal();
  int comm_ret = decodeSignalFromComms();
  if(startGyroNegative && (comm_ret == 0)){
    makeMotorsSpinCC();
  } else if(!startGyroNegative && (comm_ret == 1)){
    makeMotorsSpinCL();
  } else{
    state = ORIENTATION_STRAIGHT;
    makeMotorsStop(); // ??? stop before backing up
    fake_timer = millis();
  }
}


void RespOrientationStraight(){
  makeMotorSpeedFast();

  makeMotorsJiggleBackwards();
 
  // go backward for 2 seconds
  if( millis() - fake_timer >= 2000){
    // go backward
    state = SLAM_LEFT;
    // start slamming
    makeMotorsStop();
    delay(50);
    makeMotorsMoveLeft();
    // this is going left to slam left
    fake_timer = millis();

    // NOTE: WHEN SLAMMING LEFT, MIGHT NEED TO MAKE
    // FRONT MOTOR STRONGER THAN OTHERS TO KEEP AGAINST WALL
  }
  
}


void RespSlamLeft(){
    // if hit limit siwtch ??
    // we are currently at the very back of box
    //makeMotorSpeedFast();
    //makeMotorsMoveForward();
    //was tape sensor c
    /*if (ReadTapeSensor_B_2()){
      //state = DONE;
      makeMotorsStop();
      fake_timer = millis();
      // we have found the first out of box line
    }
    */

    if(millis() - fake_timer > TIME_TO_SLAM_LEFT){
      state = EXTEND_EYE;
      // extend tape
    }
    
}

void RespExtendEye(){
  delay(1000);
  // ????? will add code eventually to drop other sensor here.
  
  // this happens after drop eye
  state = LOOK_FOR_FIRST_TAPE;
  fake_timer = millis();
  makeMotorsMoveForward();
}

void RespLookForFirstTape(){
    // stop at first tape
    if(ReadTapeSensor_EYE()){
      // we have found first tape
      // could slow motors here
    }
}

void RespFoundFirstTape(){
  if(!ReadTapeSensor_EYE()){
    // we no longer read it, must be at the end here.
    makeMotorsStop();
    // setup shooter things
  }
}

void RespShootFromFirst(){
  // do all the shooting things here
  delay(2000);
  // TODO: ???? take out and put in the shoot boolean thing

  //WHEN DONE SHOOTING:
  makeMotorsMoveForward();
  state = LOOK_FOR_SECOND_TAPE;
}

void RespLookForSecondTape(){
  if(ReadTapeSensor_EYE()){
    // could make motors go slower here
    state = FOUND_SECOND_TAPE;
  }  
}

void RespFoundSecondTape(){
  if(!ReadTapeSensor_EYE()){
    // at the end of the tape
    makeMotorsStop();
    // setup shoot stuff
    state = SHOOT_FROM_SECOND;
  }
}


void RespDone(){
 // page left intentionally blank
}

int decodeSignalFromComms(){
  int commsState = map(pwm_value_comms, 0, 1920, 0, 11);
  /*
   * 0 - gyro negative
   * 1 - gyro positive
   */
  return commsState;
}


/* ====================================================================
 *  helper functions to communicate with motor arduino
 * ====================================================================
 */

void makeMotorsMoveForward(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_FORWARD);
}

void makeMotorsMoveBackward(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_BACKWARD);
}
 
void makeMotorSpeedSlow(){
  analogWrite(PIN_OUTPUT_SPEED_CONTROL, COMM_MOTOR_SLOW); 
}

void makeMotorSpeedNormal(){
  analogWrite(PIN_OUTPUT_SPEED_CONTROL,COMM_MOTOR_NORMAL); 
}

void makeMotorSpeedFast(){
  analogWrite(PIN_OUTPUT_SPEED_CONTROL,COMM_MOTOR_FAST);
}

void makeMotorsStop(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_STOP);
}

void makeMotorsCoastStop(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL,COMM_MOTOR_COAST_STOP); 
}

void makeMotorsSpinCL(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CL);
}

void makeMotorsSpinCC(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CC);
}

void makeMotorsMoveRight(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_RIGHT);
}

void makeMotorsMoveLeft(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_LEFT);
}

void makeMotorsJiggleBackwards(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_BACKWARD_JIGGLE);
}

/* ====================================================================
 *  ReadTapeSensor functions
 * ====================================================================
 */
bool ReadTapeSensor_A(void){
    // 1 if read tape, 0 if not 
    return digitalRead(PIN_SENSOR_A);
}

bool ReadTapeSensor_B_1(void){
    // 0 if read tape, 1 if not 
    // CHECKED
    return digitalRead(PIN_SENSOR_B_1);
}

bool ReadTapeSensor_B_2(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_B_2);
}

bool ReadTapeSensor_C(void){
    // 1 if read tape, 0 if not
    // CHECKED 
    return !digitalRead(PIN_SENSOR_C);
}

bool ReadTapeSensor_D_1(void){
    // 0 if read tape, 1 if not 
    return digitalRead(PIN_SENSOR_D_1);
}

bool ReadTapeSensor_D_2(void){
    // 1 if read tape, 0 if not 
    return digitalRead(PIN_SENSOR_D_2);
}

bool ReadTapeSensor_E(void){
    // 1 if read tape, 0 if not 
    // CHECKED
    return digitalRead(PIN_SENSOR_E);
}

bool ReadTapeSensor_EYE(void){
  // ????? TODO
  return true;
}

void SetupPins(){
  pinMode(PIN_SENSOR_A, INPUT_PULLUP);
  pinMode(PIN_SENSOR_B_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_B_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_C, INPUT);
  pinMode(PIN_SENSOR_D_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_D_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_E, INPUT_PULLUP);
  
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  pinMode(PIN_OUTPUT_SPEED_CONTROL, OUTPUT);
  //pinMode(PIN_INPUT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_GAME_START, INPUT_PULLUP);


}

void findFreqComms(){
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), freqCountComms, FALLING);
  prev_time_comms = micros();
}

void freqCountComms(){
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
  pwm_value_comms = micros() - prev_time_comms;
}
