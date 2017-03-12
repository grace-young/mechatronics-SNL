#include <Timers.h>

// ------ TAPE SENSOR PINS -------
#define PIN_SENSOR_D_1 A3
#define PIN_SENSOR_B_2 A4 // NEW
#define PIN_SENSOR_D_2 13 //Not Working
#define PIN_SENSOR_F   12

// ----- COMMM PINS --------
#define PIN_COMMS_IN_GYRO        2
 
#define PIN_OUTPUT_SPEED_CONTROL 9
#define PIN_OUTPUT_MOTOR_CONTROL 10
#define PIN_COMMS_SHOOT          5 //7 from shoot
#define PIN_COMMS_SHOOT_DONE     6 //8 from shoot
#define PIN_COMMS_SERVO          7 //10 from shoot
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
#define TIME_INTERVAL             500
#define JIGGLE_INTERVAL           400
#define TIME_INTERVAL_2           1000
#define TIME_BETWEEN_LINE_TAPE_READS  500
#define TIME_MOVE_RIGHT           5000
#define TIME_TO_PIVOT             250
#define TIME_TO_SLAM_LEFT         2000
#define TIME_TO_HEAD_BACK         4000
#define TIME_TO_SLAM_RIGHT        3000
#define TIME_TO_SLAM_PRESSURE     3000
#define TIME_TO_SLAM_DOWN         4000
#define TIME_TO_RELOAD            8000

#define TIME_WAIT_AFTER_SHOOT 3000

#define TIME_TO_WAIT_FOR_TAPE 100

#define TIME_TO_WAIT_TIL_FIRST_TAPE 1500

#define TIME_TO_PAUSE      1250

#define TIME_TO_WAIT_TIL_SEE_D    150
#define TIME_WAIT_TIL_SEE_EYE     70

#define GAME_TIME         130000


#define HOW_MANY_RUNS     3

// ---- CONSTANTS ----
static bool startGyroNegative = false;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;

static int numRuns = 1;



// ---- STATES -----
typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT,
  AT_BACK_OF_BOX, SLAM_LEFT, LOOK_FOR_FIRST_TAPE,
  FOUND_FIRST_TAPE, SHOOT_FROM_FIRST, LOOK_FOR_SECOND_TAPE, 
  FOUND_SECOND_TAPE, SHOOT_FROM_SECOND, HEADING_BACK, 
  SLAM_RIGHT, SLAM_PRESSURE, SLAM_DOWN, LOOK_FOR_BOX_TAPE, 
  LOOK_FOR_BOX_TAPE_WITH_EYE, BACK_TO_RELOAD, DONE,
  JUMP_SHOOT
  
} States_t;

States_t state;

static bool arm_already_in_beginning = false;

void setup() {
  Serial.begin(57600);

  SetupPins();
  makeMotorSpeedNormal();

  // this is for reading the gyro
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
  fake_timer = millis();
//  disarmEye();
  
  state = WAIT;
}

void loop() {
 //Serial.println(digitalRead(PIN_SENSOR_F));
  CheckGlobalStates();
}

void CheckGlobalStates(void){
  Serial.println(state);
  switch (state){
    case WAIT: //0
      RespWait();
      break;
    case START: //1
      RespStart();
      break;
    case ORIENTATION_STRAIGHT: //2 
      RespOrientationStraight();
      break;
    case SLAM_LEFT://4
      RespSlamLeft();
      // need to test here
      break;
    case LOOK_FOR_FIRST_TAPE: //5
      RespLookForFirstTape();
      break;
    case FOUND_FIRST_TAPE: //6
      RespFoundFirstTape();
      break;
    case SHOOT_FROM_FIRST: //7
      RespShootFromFirst();
      break;
    case LOOK_FOR_SECOND_TAPE: //8
      RespLookForSecondTape();
      break;
    case FOUND_SECOND_TAPE: //9
      RespFoundSecondTape();
      break;
    case SHOOT_FROM_SECOND: //10
      RespShootFromSecond();
      break;
    case HEADING_BACK: //11
      RespHeadingBack();
      break;
    case SLAM_RIGHT: //12
      RespSlamRight();
      break;
    case SLAM_PRESSURE: //13
      RespSlamPressure();
      break;
    case SLAM_DOWN: //14
      RespSlamDown();
      break;
    case LOOK_FOR_BOX_TAPE: //15
      RespLookForBoxTape();
      break;
    case LOOK_FOR_BOX_TAPE_WITH_EYE: //16
      RespLookForBoxTapeWithEye();
      break;
    case BACK_TO_RELOAD: //17
      RespBackToReload();
      break;
    case DONE: //18
      RespDone();
      break;
    case JUMP_SHOOT: //19
      RespJumpShoot();
      break;
    default:
      break;
  }
}

void RespWait(){
  makeMotorsCoastStop();

  if(!arm_already_in_beginning){
    arm_already_in_beginning = true;
    disarmEye();
  }
  
  if(decodeSignalFromComms() == 0){
      startGyroNegative = true;
  }else{
      startGyroNegative = false;
  }
  delay(150); // ????? might not need
  if(!digitalRead(PIN_GAME_START)){
    state = START;
    //state = DONE;
    //state = JUMP_SHOOT;
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
  makeMotorSpeedFast(); // this might be an issue?????

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

    if(millis() - fake_timer > TIME_TO_SLAM_LEFT){
      makeMotorsStop();
      armEye();
      delay(TIME_TO_WAIT_TIL_FIRST_TAPE);
      makeMotorsMoveForward();
      state = LOOK_FOR_FIRST_TAPE;
    }
}


void RespLookForFirstTape(){
    // stop at first tape
    if(ReadTapeSensor_EYE()){
      makeMotorSpeedSlow(); ``````````````
      state = FOUND_FIRST_TAPE;
      fake_timer = millis();
    }
}

void RespFoundFirstTape(){
  if(!ReadTapeSensor_EYE() && millis() - fake_timer > TIME_TO_WAIT_FOR_TAPE){
    // we no longer read it, must be at the end here.
    makeMotorsStop();
    shootOn();
    state = SHOOT_FROM_FIRST;
    fake_timer = millis();
  }
}

void RespShootFromFirst(){
  // do all the shooting things here
  //delay(2000);
  // TODO: ???? take out and put in the shoot boolean thing
  if(shootDone() && millis() - fake_timer > TIME_WAIT_AFTER_SHOOT){
    //state=DONE;
    makeMotorSpeedFast();
    makeMotorsMoveForward();
    state = LOOK_FOR_SECOND_TAPE;
    fake_timer = millis();
  }
}

void RespLookForSecondTape(){
  if(ReadTapeSensor_EYE() && millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
    // could make motors go slower here
    makeMotorSpeedSlow();
    state = FOUND_SECOND_TAPE;
    fake_timer = millis();
  }  
}

void RespFoundSecondTape(){
  if(!ReadTapeSensor_EYE() && millis() - fake_timer > TIME_TO_WAIT_FOR_TAPE){
    // at the end of the tape
    makeMotorsStop();
    shootOn();
    state = SHOOT_FROM_SECOND;
    fake_timer = millis();
  }
}

void RespShootFromSecond(){
  // do all the shooting things here
  if(shootDone() && millis() - fake_timer > TIME_WAIT_AFTER_SHOOT){
    makeMotorSpeedFast();
    makeMotorsMoveBackward();
  // get timer, but could become limit switch thing later in next
  // state
  fake_timer = millis();
  state = HEADING_BACK;
  }
}

void RespHeadingBack(){
  if(millis()-fake_timer > TIME_TO_HEAD_BACK){
    // now we are about to slam right against wall
    // because we are back in back left corner by
    // safe space
    disarmEye();
    makeMotorsMoveRight();
    fake_timer = millis();
    state = SLAM_RIGHT;
  }
}

void RespSlamRight(){
  if(millis() - fake_timer > TIME_TO_SLAM_RIGHT){
    // now going to slam up into the block
    // this will also hit the pressuer sensor
    makeMotorSpeedFast();
    makeMotorsMoveForward();
    fake_timer = millis();
    state = SLAM_PRESSURE;
  }
}

void RespSlamPressure(){
  // all these slamming if statements
  // could be replaced with LIMIT SWITHCES
  if(millis() - fake_timer > TIME_TO_SLAM_PRESSURE){
    // now we've hopefully hit that block
    // go back down to slam into bottom right corner
    makeMotorsMoveBackward();
    fake_timer = millis();
    state = SLAM_DOWN;
  }
}

void RespSlamDown(){
  if(millis() - fake_timer > TIME_TO_SLAM_DOWN){
    // now we're in bottom right corner
    // now we go left til we read sensor on the right side
    makeMotorsMoveLeft();
    state = LOOK_FOR_BOX_TAPE;
    fake_timer = millis();
  }
}

void RespLookForBoxTape(){
  // go until we read tape D which means we are almost
  // all the way in the box
  if((ReadTapeSensor_D_1()) && millis() - fake_timer > TIME_TO_WAIT_TIL_SEE_D){
    // stop motors
    makeMotorsStop();
    armEye();
    state = LOOK_FOR_BOX_TAPE_WITH_EYE;
    makeMotorsMoveRight();
    fake_timer = millis();
  }
}

void RespLookForBoxTapeWithEye(){
  if(ReadTapeSensor_EYE() && millis() - fake_timer > TIME_WAIT_TIL_SEE_EYE){
    // we are at edge of box then ???
    // LIFT THE EYE HERE
    makeMotorsStop();
    disarmEye();
    state = BACK_TO_RELOAD;
    fake_timer  = millis();
  }
}

void RespBackToReload(){
  if(millis()-fake_timer > TIME_TO_RELOAD){
    // ALL BALLS SHOULD BE RELOADED NOW
    
    if(numRuns < HOW_MANY_RUNS){
      // DO IT ALL AGAIN
      numRuns++;
      state = ORIENTATION_STRAIGHT;
      fake_timer = millis();
    }else{
      state = DONE;
    }

  }
}


void RespJumpShoot(){
    // at the end of the tape
    makeMotorsStop();
    shootOn();
    if(shootDone()){
      state = DONE;
      makeMotorsStop();
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
 * Helper Functions to communicate with Mini
 * ====================================================================
 */
 void shootOn () {
    digitalWrite(PIN_COMMS_SHOOT, HIGH);
    delay(50);
    digitalWrite(PIN_COMMS_SHOOT, LOW);
 }

 bool shootDone(){
    if(!digitalRead(PIN_COMMS_SHOOT_DONE)){
      return true;
    }else{
      return false;
    }
 }

 void armEye() {
  digitalWrite(PIN_COMMS_SERVO,HIGH);
 }

 void disarmEye() {
  digitalWrite(PIN_COMMS_SERVO,LOW);
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

bool ReadTapeSensor_B_2(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_B_2);
}

bool ReadTapeSensor_D_1(void){
    // 0 if read tape, 1 if not 
    return digitalRead(PIN_SENSOR_D_1);
}

bool ReadTapeSensor_D_2(void){
    // 1 if read tape, 0 if not 
    return digitalRead(PIN_SENSOR_D_2);
}

bool ReadTapeSensor_EYE(void){
  // ????? TODO
  return digitalRead(PIN_SENSOR_F);
}

void SetupPins(){
  pinMode(PIN_SENSOR_B_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_D_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_D_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_F, INPUT_PULLUP);

  pinMode(PIN_COMMS_SHOOT, OUTPUT);
  pinMode(PIN_COMMS_SHOOT_DONE, INPUT);
  pinMode(PIN_COMMS_SERVO, OUTPUT);
  digitalWrite(PIN_COMMS_SHOOT,LOW);
  
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
