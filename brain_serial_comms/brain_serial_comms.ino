#include <Timers.h>
#include <Pulse.h>

// -------- PIN NUMBERS ----------
#define PIN_SENSOR_A 7 
#define PIN_SENSOR_B_1 5
#define PIN_SENSOR_C 8
#define PIN_SENSOR_D_1 13
#define PIN_SENSOR_E 12
#define PIN_SENSOR_B_2 A0 // NEW
#define PIN_SENSOR_D_2 A5 //11 

#define PIN_RED2_DEBUG A2
#define PIN_RED_DEBUG A3
#define PIN_GREEN_DEBUG A4

#define PIN_GAME_START           11 

#define LED1  9
#define LED2  10
#define LED3  A4
#define LED4  A3
#define LED5  A2

// rip to yuji's comms

/* =====================================
 *  STEPPER & FLYWHEEL CONSTANTS
 * =====================================
 */
#define PULSE_STEPPER 4
#define FLYWHEEL_ON A1
#define STEPPER_PERIOD 200
#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 1000

bool shoot = false;
bool pulses_done = false;
bool calibrate_tick = true;

static int NUM_PULSES = 29;
const int CALIBRATE_COUNT = 4;
static int count = 1;

static int shots_fired = 0;
static int n = 0;
static int numLinesCounted = 0;
static int  shoot_count;

// ------ GYRO THINGS --------
static bool startGyroNegative = false;

/* =====================================
 *  SERIAL COMMS CONSTANTS
 * =====================================
 */
const char COMM_MOTOR_NORMAL  = 'a';
const char COMM_MOTOR_SLOW = 'b';
const char COMM_MOTOR_FAST = 'c';
const char COMM_ZERO_GYRO = 'd';

const char COMM_MOTOR_COAST_STOP = 'e';
const char COMM_MOTOR_FORWARD = 'f';
const char COMM_MOTOR_BACKWARD = 'g';
const char COMM_MOTOR_LEFT = 'h';
const char COMM_MOTOR_RIGHT = 'i';
const char COMM_MOTOR_SPIN_CC = 'j';
const char COMM_MOTOR_SPIN_CL = 'k';
const char COMM_MOTOR_STOP = 'l';
const char COMM_MOTOR_BACKWARD_JIGGLE = 'm';
// =========================================

//commands to Brain 
const char GYRO_NEGATIVE              = 'n';
const char GYRO_POSITIVE              = 'o';

// -------- FAKE TIMERS ----------
static unsigned long fake_timer;
static unsigned long stepper_timer;

// -------- TIMER INTERVALS ------
#define TIME_INTERVAL      500
#define JIGGLE_INTERVAL    400
#define TIME_INTERVAL_2    1000
#define TIME_BETWEEN_LINE_TAPE_READS  500
#define TIME_MOVE_RIGHT    5000
#define TIME_TO_PIVOT      250
#define TIME_TO_PAUSE      1250

// more comms things:
#define PIN_COMMS_OUT_HERE_TX  1

#define PIN_GYRO           9
typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT,
  AT_BACK_OF_BOX, CROSSED_FIRST_LINE, 
  SHOOT_ON_SECOND_LINE, LOOKING_FOR_THIRD_LINE, 
  SHOOT_ON_THIRD_LINE, SHOOT_DONE
} States_t;

States_t state;


static bool alignLeft;

void setup() {
  SetupPins();

  Serial.begin(57600);
  
  state = WAIT;
    
  alignLeft=true;
  
  // unclear if we need this
  makeMotorSpeedNormal();
  fake_timer = millis();
}


void loop() {
    // we know the tape sensor values are right noe
    CheckGlobalStates();
}

void CheckGlobalStates(void){
  switch (state){
    case WAIT:
      RespToWait();
      break;
    case START:
      RespToStart();
      break;
    case ORIENTATION_STRAIGHT:
      RespOrientationStraight();
      break;
    case AT_BACK_OF_BOX:
      RespAtBackOfBox();
      break;
    case CROSSED_FIRST_LINE:
      RespCrossedFirstLine();
      break;
    case SHOOT_ON_SECOND_LINE:
      RespShootOnSecondLine();
      break;
    case LOOKING_FOR_THIRD_LINE:
      RespLookingForThirdLine();
      break;
    case SHOOT_ON_THIRD_LINE:
      RespShootOnThirdLine();
      break;
    case SHOOT_DONE:
      RespShootDone();
      break;
    default:
      break;
  }
}

int decodeSignalFromComms(){

  char comm_state = 'a';
  switch (comm_state){
    case GYRO_NEGATIVE:
        return 0;
    case GYRO_POSITIVE:
        return 1;
    default:
      return 0;
  }
}

void RespAtBackOfBox(){
    makeMotorSpeedNormal();
    makeMotorsMoveForward();
    //was tape sensor c
    if (ReadTapeSensor_B_1()){
      state = CROSSED_FIRST_LINE;
      fake_timer = millis();
    }
}


void RespCrossedFirstLine(){
  if(millis()-fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
    if(ReadTapeSensor_B_1()){
      // we are on the second line
      state = SHOOT_ON_SECOND_LINE;
      makeMotorsStop();
    }
  }
  
}

void RespShootOnSecondLine(){
  if(shoot_count < 3){
    respondToShoot();
  }else {
    // we are done shooting -- hopefully!!
    endShoot();
    state = LOOKING_FOR_THIRD_LINE;
    shoot_count = 0;
    makeMotorSpeedNormal();
    makeMotorsMoveForward();
    fake_timer = millis(); // set timer
  }
}

void respondToShoot() {
    digitalWrite(FLYWHEEL_ON, HIGH);
    if(IsPulseFinished()){
      //pulses_done = true;
    }
    if(millis() - stepper_timer > STEPPER_TIME_INTERVAL){
      shoot_count ++;
      delay(1000);
      //setupPulses();
    }
}

void setupPulses(){
  EndPulse(); // stops the old speed pulses 
  InitPulse(PULSE_STEPPER, STEPPER_PERIOD); // set new pulses
  count++;
  shoot_count++;
  if (count == CALIBRATE_COUNT && calibrate_tick) {
    calibrate_tick = false;
    count = 1;
    NUM_PULSES--;
  } else if (count == CALIBRATE_COUNT && !calibrate_tick) {
    calibrate_tick = true;
    count = 1;
    NUM_PULSES++;
  }
  Pulse(NUM_PULSES); 
  pulses_done = false;
  stepper_timer = millis();
//  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
}

void endShoot(){
  digitalWrite(FLYWHEEL_ON, LOW);
  EndPulse();
}


void RespLookingForThirdLine(){
  if(millis()-fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
    if(ReadTapeSensor_B_1()){
      // we see a third line
      makeMotorsStop();
      state = SHOOT_ON_THIRD_LINE;
    }
  }
}
void RespShootOnThirdLine(){
  if(shoot_count < 3){
    respondToShoot();
  }else {
    // we are done shooting -- hopefully!!
    endShoot();
    state = SHOOT_DONE;
    shoot_count = 0;
    makeMotorSpeedNormal();
    makeMotorsMoveRight();
    fake_timer = millis(); // set timer to count how long we go right
  }

}
void RespShootDone(){;
  if(millis()-fake_timer > TIME_MOVE_RIGHT){
    makeMotorsStop();
    // robot should be against the wall now.
  }
}

void RespCountedThreeLines(){
  makeMotorsMoveRight(); // send only when state changes
}

void RespToWait(){
  makeMotorsCoastStop();
  /*
   * 0 - gyro negative
   * 1 - gyro positive
   */
  
  if(decodeSignalFromComms() == 0){
      startGyroNegative = true;
  }else{
      startGyroNegative = false;
  }
  if(!digitalRead(PIN_GAME_START)){
    state = START;
  }
}

void RespToStart(){
    // speed used to be slow but now normal
   //analogWrite(PIN_OUTPUT_SPEED_CONTROL, COMM_MOTOR_NORMAL); 
   makeMotorSpeedNormal();
  /*
   * 0 - gyro negative
   * 1 - gyro positive
   */
   int comm_ret = decodeSignalFromComms();

  if(startGyroNegative && !ReadIfGyroPositive()){
    makeMotorsSpinCC();
  } else if(!startGyroNegative && ReadIfGyroPositive()){
    makeMotorsSpinCL();
  } else{
    state = ORIENTATION_STRAIGHT;
    fake_timer = millis();
  }
}

void RespOrientationStraight(){
  // stop

  makeMotorSpeedFast();
// do we need delay's here???

  makeMotorsJiggleBackwards();
 
  // go backward for 2 seconds
  if( millis() - fake_timer >= 2000){
    // go backward
    state = AT_BACK_OF_BOX;
    fake_timer = millis();
  }
}


void RespBlock(){
  if(millis()-fake_timer > TIME_MOVE_RIGHT){
    //state = DONE;
    makeMotorsStop();
  }
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

bool ReadIfGyroPositive(void){
    return digitalRead(PIN_GYRO);
}

/* ====================================================================
 *  helper functions to communicate with motor arduino
 * ====================================================================
 */

void makeMotorsMoveForward(){
  Serial.print(COMM_MOTOR_FORWARD);
}

void makeMotorsMoveBackward(){
  Serial.print(COMM_MOTOR_BACKWARD);
}
 
void makeMotorSpeedSlow(){
  Serial.print(COMM_MOTOR_SLOW);
}

void makeMotorSpeedNormal(){
  Serial.print(COMM_MOTOR_NORMAL);
}

void makeMotorSpeedFast(){
  Serial.print(COMM_MOTOR_FAST);
}

void makeMotorsStop(){
  Serial.print(COMM_MOTOR_STOP);
}

void makeMotorsCoastStop(){
  Serial.print(COMM_MOTOR_COAST_STOP);
}

void makeMotorsSpinCL(){
  Serial.print(COMM_MOTOR_SPIN_CL);
}

void makeMotorsSpinCC(){
  Serial.print(COMM_MOTOR_SPIN_CC);
}

void makeMotorsMoveRight(){
  Serial.print(COMM_MOTOR_RIGHT);
}

void makeMotorsMoveLeft(){
  Serial.print(COMM_MOTOR_LEFT);
}

void makeMotorsJiggleBackwards(){
  Serial.print(COMM_MOTOR_BACKWARD_JIGGLE);
}

/* ====================================================================
 *  SETUP PINS function
 * ====================================================================
 */

void SetupPins(){
  pinMode(PIN_SENSOR_A, INPUT);
  pinMode(PIN_SENSOR_B_1, INPUT);
  pinMode(PIN_SENSOR_B_2, INPUT);
  pinMode(PIN_SENSOR_C, INPUT);
  pinMode(PIN_SENSOR_D_1, INPUT);
  pinMode(PIN_SENSOR_D_2, INPUT);
  pinMode(PIN_SENSOR_E, INPUT);

  pinMode(PIN_RED_DEBUG, OUTPUT);
  pinMode(PIN_GREEN_DEBUG, OUTPUT);
  pinMode(PIN_RED2_DEBUG, OUTPUT);
  

  pinMode(PIN_GYRO, INPUT);
  
  pinMode(PIN_COMMS_OUT_HERE_TX, OUTPUT);
  
  pinMode(PIN_GAME_START, INPUT_PULLUP);

  pinMode(PULSE_STEPPER, OUTPUT);
  pinMode(FLYWHEEL_ON, OUTPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
}

// rip gracie's 32 states
