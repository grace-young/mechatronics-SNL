#include <Timers.h>

// ------ TAPE SENSOR PINS -------
#define PIN_SENSOR_D_1 A3
#define PIN_SENSOR_B_2 11 // NEW
//#define PIN_SENSOR_D_2 13 //Not Working
#define PIN_SENSOR_F   12
#define PIN_SENSOR_F_2 13

// ------ LIMIT SWITCHES --------
#define PIN_LEFT_LIMIT A0
#define PIN_BACK_LIMIT A1

#define PIN_BALL_COMMAND_DONE_LOADING 4
#define PIN_BALL_COMMAND_SHOOT1       A4
#define PIN_BALL_COMMAND_SHOOT2       A5
#define PIN_BALL_COMMAND_SHOOT3       A2

// ----- COMMM PINS --------
#define PIN_COMMS_IN_GYRO        2
 
#define PIN_OUTPUT_SPEED_CONTROL 9
#define PIN_OUTPUT_MOTOR_CONTROL 10
#define PIN_COMMS_SHOOT          5 //7 from shoot
#define PIN_COMMS_SHOOT_DONE     6 //8 from shoot
#define PIN_COMMS_SERVO          7 //10 from shoot
#define PIN_GAME_START           8 // same
#define PIN_RESET_BODY           3

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
#define TIME_TO_HEAD_BACK         10000
#define TIME_TO_RELOAD            8000
#define TIME_TO_RELOAD_MIN        5000
#define TIME_SHIFT_IN             2000
#define TIME_TO_JIGGLE            1500

#define TIME_TO_SCOOT_OUT         250

#define TIME_WAIT_AFTER_SHOOT 3000

#define TIME_TO_WAIT_FOR_TAPE 100

#define TIME_GO_RIGHT_INTO_BOX    3000

#define TIME_TO_PAUSE      1250

#define TIME_TO_WAIT_TIL_SEE_D    150
#define TIME_WAIT_TIL_SEE_EYE     70

#define GAME_TIME         130000


// backup time checks
#define BACKUP_TIME_LOOKING_FOR_END_TAPE 5000

#define HOW_MANY_RUNS     3

// ---- CONSTANTS ----
static bool startGyroNegative = false;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;
static unsigned long game_timer;

static int numRuns = 1;

static bool loaded_first_already = false;
static int aimTower = -1;
static int tapeCtr = 0;

// ---- STATES -----
typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT,
  SLAM_LEFT, SCOOT_OUT, LOOK_FOR_FIRST_TAPE,
  FOUND_FIRST_TAPE, SHOOT_FROM_FIRST, LOOK_FOR_SECOND_TAPE, 
  FOUND_SECOND_TAPE, SHOOT_FROM_SECOND, LOOK_FOR_THIRD_TAPE,
  FOUND_THIRD_TAPE, SHOOT_FROM_THIRD, SHIFT_IN,
  HEADING_BACK, SCOOT_INTO_BOX, MOSTLY_IN_BOX, RELOAD,
  JUST_JIGGLE, JUST_SLAM_LEFT,LOOK_FOR_TAPE_START,
  LOOK_FOR_STATE_END,
  BACK_TO_RELOAD, DONE,
  JUMP_SHOOT
  
} States_t;

States_t state;

static bool arm_already_in_beginning = false;

void setup() {
  Serial.begin(9600);

  SetupPins();
  makeMotorSpeedNormal();

  // this is for reading the gyro
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
  fake_timer = millis();
  resetBody();
  disarmEye();
  
  state = WAIT; 
}

void loop() {
//  makeMotorSpeedFast();
//  makeMotorsMoveForward();
  if(millis()-game_timer >= GAME_TIME ){
    // END HERE time is up
    state = DONE;
  }
  CheckGlobalStates();
  //Serial.println(ReadTapeSensor_EYE());
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
    case SCOOT_OUT:
      RespScootOut();
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
    case LOOK_FOR_THIRD_TAPE: //11
      RespLookForThirdTape();
      break;
    case FOUND_THIRD_TAPE: //12
      RespFoundThirdTape();
      break; 
    case SHOOT_FROM_THIRD: //13
      RespShootFromThird();
      break;
    case SHIFT_IN: //14
      RespShiftIn();
      break;
    case HEADING_BACK: //15
      RespHeadingBack();
      break;
    case SCOOT_INTO_BOX: //16
      RespScootIntoBox();
      break;
    case MOSTLY_IN_BOX: //17
      RespMostlyInBox();
      break;
    case RELOAD: //18
      RespReload();
      break;
    case JUST_JIGGLE://19
      RespJustJiggle();
      break;
    case JUST_SLAM_LEFT: //20
      RespJustSlamLeft();
      break;
    case LOOK_FOR_TAPE_START: //21
      RespLookForTapeStart();
      break;
    case LOOK_FOR_STATE_END: //22
      RespLookForStateEnd();
      break;
    case BACK_TO_RELOAD: //23
      RespBackToReload();
      break;
    case DONE: //24
      RespDone();
      break;
    case JUMP_SHOOT: //25
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
  if(backBumperHit() || millis() - fake_timer >= 2000){//add some small time component as well to account for random crap millis() - fake_timer >= 2000
    // go backward
    state = SLAM_LEFT;
    // start slamming
    makeMotorsStop();
    delay(50);
    makeMotorSpeedFast();
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

    if(leftBumperHit()||millis() - fake_timer > TIME_TO_SLAM_LEFT ){//add a timer componenet to this in case of ball jamming occuring. millis() - fake_timer > TIME_TO_SLAM_LEFT 
      makeMotorsStop();
      resetBody();
      armEye();
      
      delay(400);//1000 before
      makeMotorSpeedFast();
      makeMotorsMoveForward();
      state = LOOK_FOR_FIRST_TAPE;
      //resetBody();
//      delay(250);
//      makeMotorsMoveRight();

//      state = SCOOT_OUT;
      fake_timer = millis();
    }
}


void RespScootOut(){
  if(millis()-fake_timer > TIME_TO_SCOOT_OUT){
    makeMotorsStop();
    resetBody();
    delay(10);

    state = LOOK_FOR_FIRST_TAPE;
  }
}

void RespLookForFirstTape(){
    // stop at first tape
    makeMotorSpeedFast();
    makeMotorsMoveForward();
    if(ReadTapeSensor_EYE()){
      makeMotorSpeedSlow();
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
    resetBody();
    delay(10);
    makeMotorSpeedFast();
    makeMotorsMoveForward();
    state = LOOK_FOR_SECOND_TAPE;
  }
}

void RespLookForSecondTape(){
  if(ReadTapeSensor_EYE()){
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
    resetBody();
    makeMotorSpeedFast();
    makeMotorsMoveForward(); // lol this was backwards before
    state = LOOK_FOR_THIRD_TAPE;
    fake_timer = millis();
  }
}


void RespLookForThirdTape(){
  if(ReadTapeSensor_EYE() || millis()-fake_timer > BACKUP_TIME_LOOKING_FOR_END_TAPE){
    makeMotorSpeedSlow();
    
    state = FOUND_THIRD_TAPE;
    fake_timer = millis();
  }
}

void RespFoundThirdTape(){
  if((!ReadTapeSensor_EYE() && millis() - fake_timer > TIME_TO_WAIT_FOR_TAPE)
    || millis() - fake_timer > BACKUP_TIME_LOOKING_FOR_END_TAPE){
    // at the end of the tape
    makeMotorsStop();
    shootOn();
    state = SHOOT_FROM_THIRD;
    fake_timer = millis();
  }
}


void RespShootFromThird(){
  if(shootDone() && millis() - fake_timer > TIME_WAIT_AFTER_SHOOT){
    resetBody();
    makeMotorSpeedFast();
    makeMotorsMoveRight();
  
    fake_timer = millis();
    // comment out later
    //makeMotorsStop();
    //state = DONE;
    state = SHIFT_IN;
  }
}

void RespShiftIn(){
  if(millis() - fake_timer > TIME_SHIFT_IN){
    makeMotorsStop();
    resetBody();
    delay(10);
    makeMotorsMoveBackward();
    fake_timer = millis();
    state = HEADING_BACK;
  }
}



void RespHeadingBack(){
  if(backBumperHit() || millis() - fake_timer > TIME_TO_HEAD_BACK){
      //add some time component to this to make sure that balls 
      //that get stuck won't fuck us over 
      // now we are about to slam right against wall
      // because we are back in back left corner by
      // safe space
    makeMotorsStop();
    resetBody();
    delay(10);
    makeMotorSpeedFast();
    makeMotorsMoveRight();
    
    // comment out later:
        //makeMotorsStop();
        //state = DONE;
    
    fake_timer = millis();
    state = SCOOT_INTO_BOX;
  }
}


void RespScootIntoBox(){
  // maybe add backup time here ???????
  if(millis()-fake_timer > TIME_GO_RIGHT_INTO_BOX){
    // ReadTapeSensor_B() || 
    makeMotorSpeedSlow(); 
    // extend eye if we need to 
    // but maybe we are no longer disarming the eye????
    state = MOSTLY_IN_BOX;
  }
}


void RespMostlyInBox(){
  // maybe add backup time here ???????
  if(ReadTapeSensor_EYE()){
    disarmEye(); // this might need to go after stop
    // but don't want to have to wait for the rest arduino
    makeMotorsStop();
    loaded_first_already = false;
    aimTower = -1;
    state = RELOAD;
    //state = DONE;
    resetBody();
    delay(10);
    fake_timer = millis();
  }
}

void RespReload(){
  // need to check which ones are pressed
  // need a way to tell which is pressed first

  // need to make sure we load them slow enough for
  // us to pick up which was actually loaded first
  if(DoneBumperLoaded() && millis()-fake_timer > TIME_TO_RELOAD_MIN){
    
    makeMotorSpeedFast();
    makeMotorsMoveLeft();
    fake_timer = millis();
    state = SLAM_LEFT;
  }
//  if(!loaded_first_already){
//    if(TowerOneBumperLoaded()){
//      aimTower = 1;
//    }else if(TowerTwoBumperLoaded()){
//      aimTower = 2;
//    }else{
//      aimTower = 3;
//    }
//    loaded_first_already = true;
//  }else{
//    aimTower = 1; // make a default at 1 just in case
//  }
//
//  if(DoneBumperLoaded() && millis()-fake_timer > TIME_TO_RELOAD_MIN){
//    state = JUST_JIGGLE;
//    resetBody();
//    makeMotorSpeedFast(); // this might be an issue?????
//    makeMotorsJiggleBackwards();
//    fake_timer = millis();
//  }
//  
}

void RespJustJiggle(){
  if(millis() - fake_timer > TIME_TO_JIGGLE){
    // done jiggling
    state = JUST_SLAM_LEFT;
    makeMotorsMoveLeft();
    makeMotorSpeedFast();
    fake_timer = millis();
  }
}

void RespJustSlamLeft(){
  if(leftBumperHit() || millis()-fake_timer > TIME_TO_SLAM_LEFT){//add a timer componenet to this in case of bal
      armEye();
      delay(500);//1000 before
      makeMotorSpeedFast();
      makeMotorsMoveForward();
      state = LOOK_FOR_TAPE_START;
      tapeCtr = 0;
    }
}


void RespLookForTapeStart(){
  if(ReadTapeSensor_EYE() ||  millis()-fake_timer > TIME_TO_WAIT_FOR_TAPE){
    makeMotorSpeedSlow();
    tapeCtr++;
    state = LOOK_FOR_STATE_END;
  }
}

void RespLookForStateEnd(){
  if(!ReadTapeSensor_EYE()){
    // stopped seeing tape
    if(tapeCtr == aimTower){
      makeMotorsStop();
      shootOn();
      fake_timer = millis();
      state = SHOOT_FROM_THIRD; // need to fix this shoot state
    }else{
      makeMotorSpeedFast();
      makeMotorsMoveForward();
      state = LOOK_FOR_TAPE_START;
      fake_timer = millis();
    }
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
  Serial.println("ARMING");
  digitalWrite(PIN_COMMS_SERVO,HIGH);
 }

 void disarmEye() {
  Serial.println("DIS---ARMING");
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

void resetBody(){
  digitalWrite(PIN_RESET_BODY, LOW);
  delay(30);
  digitalWrite(PIN_RESET_BODY, HIGH);
}

/* ====================================================================
 *  ReadTapeSensor functions
 * ====================================================================
 */

bool ReadTapeSensor_B(void){
    // 1 if read tape, 0 if not 
    return digitalRead(PIN_SENSOR_B_2);
}

bool ReadTapeSensor_D_1(void){
    // 0 if read tape, 1 if not 
    return digitalRead(PIN_SENSOR_D_1);
}

bool ReadTapeSensor_EYE(void){
  // ????? TODO
 // Serial.print("tape sensors: one--");
//  Serial.print(digitalRead(PIN_SENSOR_F));
//  Serial.print("  two--");
//  Serial.println(digitalRead(PIN_SENSOR_F_2));
  return digitalRead(PIN_SENSOR_F) || digitalRead(PIN_SENSOR_F_2);
}

/* ====================================================================
 *  Bumper hit Sensor functions
 * ====================================================================
 */

bool leftBumperHit(void){
  return !digitalRead(PIN_LEFT_LIMIT);
}

bool backBumperHit(void){
  return !digitalRead(PIN_BACK_LIMIT);
}

bool DoneBumperLoaded(void){
  // is this not supposed to be not???????
  return !digitalRead(PIN_BALL_COMMAND_DONE_LOADING);
}

bool TowerOneBumperLoaded(void){
  return !digitalRead(PIN_BALL_COMMAND_SHOOT1);
}

bool TowerTwoBumperLoaded(void){
  return !digitalRead(PIN_BALL_COMMAND_SHOOT2);
}

bool TowerThreeBumperLoaded(void){
  return !digitalRead(PIN_BALL_COMMAND_SHOOT3);
}



void SetupPins(){
  pinMode(PIN_SENSOR_B_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_D_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_F_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_F, INPUT_PULLUP);

  pinMode(PIN_LEFT_LIMIT, INPUT_PULLUP);
  pinMode(PIN_BACK_LIMIT, INPUT_PULLUP);
  pinMode(PIN_BALL_COMMAND_DONE_LOADING, INPUT_PULLUP);
  pinMode(PIN_BALL_COMMAND_SHOOT1, INPUT_PULLUP);
  pinMode(PIN_BALL_COMMAND_SHOOT2, INPUT_PULLUP);
  pinMode(PIN_BALL_COMMAND_SHOOT3, INPUT_PULLUP);

  pinMode(PIN_COMMS_SHOOT, OUTPUT);
  pinMode(PIN_COMMS_SHOOT_DONE, INPUT);
  pinMode(PIN_COMMS_SERVO, OUTPUT);
  digitalWrite(PIN_COMMS_SHOOT,LOW);
  pinMode(PIN_COMMS_IN_GYRO,INPUT);
  
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  pinMode(PIN_OUTPUT_SPEED_CONTROL, OUTPUT);
  //pinMode(PIN_INPUT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_GAME_START, INPUT_PULLUP);
  pinMode(PIN_RESET_BODY, OUTPUT);

}

void findFreqComms(){
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), freqCountComms, FALLING);
  prev_time_comms = micros();
}

void freqCountComms(){
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
  pwm_value_comms = micros() - prev_time_comms;
}
