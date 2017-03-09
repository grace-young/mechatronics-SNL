#include <Timers.h>
#include <Pulse.h>

#define PIN_SENSOR_A 7 
#define PIN_SENSOR_B_1 5
#define PIN_SENSOR_C 8
#define PIN_SENSOR_D_1 13
#define PIN_SENSOR_E 12
#define PIN_SENSOR_B_2 6
#define PIN_SENSOR_D_2 11

// STEPPER SHIT

#define FLYWHEEL_ON A1
#define PULSE_STEPPER 7
#define STEPPER_PERIOD 200

#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 1000


#define PIN_RED2_DEBUG A2
#define PIN_RED_DEBUG A3
#define PIN_GREEN_DEBUG A4
#define PIN_YELLOW_DEBUG A5

#define PIN_COMMS_IN_GYRO        2
#define PIN_OUTPUT_MOTOR_CONTROL 3
#define PIN_OUTPUT_SPEED_CONTROL 9
#define PIN_INPUT_BUMPER         A0

#define PIN_GAME_START           10

#define TIME_INTERVAL      500
#define JIGGLE_INTERVAL    400
#define TIME_INTERVAL_2    1000
#define TIME_BETWEEN_LINE_TAPE_READS  500
#define TIME_MOVE_RIGHT    5000
#define TIME_TO_PIVOT      250

#define TIME_TO_PAUSE      1250


#define TIMER_0            0

#define COMM_MOTOR_NORMAL  20
#define COMM_MOTOR_SLOW    40
#define COMM_MOTOR_FAST    60
#define COMM_ZERO_GYRO     80
// add the comm to reset gyro at some way
//#define COMM_MOTOR_RESET_GYRO  180


#define COMM_MOTOR_COAST_STOP 20
#define COMM_MOTOR_FORWARD 40
#define COMM_MOTOR_BACKWARD 60
#define COMM_MOTOR_LEFT 80
#define COMM_MOTOR_RIGHT 100
#define COMM_MOTOR_SPIN_CC 120
#define COMM_MOTOR_SPIN_CL 140
#define COMM_MOTOR_STOP 160
#define COMM_MOTOR_BACKWARD_JIGGLE 180

static bool startGyroNegative = false;

static bool pivot_CC;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;
static unsigned long pivot_timer;

static int shots_fired = 0;
static int n = 0;
static int numLinesCounted = 0;
static int  shoot_count;

//shooter bullshit

#define PULSE_STEPPER 4
#define FLYWHEEL_ON A2
#define STEPPER_PERIOD 200

#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 1000

bool shoot = false;
bool pulses_done = false;
bool calibrate_tick = true;

static int NUM_PULSES = 29;
const int CALIBRATE_COUNT = 4;
static int count = 1;

typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT,
  FOUND_T_LINE, AT_BACK_OF_BOX, CROSSED_ONE_LINE, CROSSED_TWO_LINES, 
  CROSSED_THREE_LINES, PAUSE_AT_LINE, ALIGN_TO_SHOOT, PIVOT_SHOOT, 
  ACTUALLY_SHOOT, ALIGN_AFTER_PIVOT, DONE, SHOOT, BLOCK, YOLO
} States_t;

States_t state;
unsigned char isTapeOn_A;
unsigned char isTapeOn_B;
unsigned char isTapeOn_C;
unsigned char isTapeOn_D;
unsigned char isTapeOn_E;

static bool alignLeft;

void setup() {
  Serial.begin(57600);
  //TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  isTapeOn_A = false;
  isTapeOn_B = false;
  isTapeOn_C = false;
  isTapeOn_D = false;
  isTapeOn_E = false;
  state = WAIT;
  
  //makeMotorsMoveForward();
  makeMotorSpeedNormal();
  SetupPins();
  
  alignLeft=true;
  pivot_CC = false;
  
  TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 

  //Interupt for reading the Comms from gyro
   attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
   fake_timer = millis();
}


void loop() {
  UpdateTapeSensorVars();
  // we know the tape sensor values are right noe
  CheckGlobalStates();
    Serial.print("numLines");
  Serial.print(numLinesCounted);
  Serial.print(" state");
  Serial.println(state);
  if(TMRArd_IsTimerExpired(TIMER_0)){
    String toprint = "state: " + (String)state + "\nA: " + (String)isTapeOn_A + " B:"+(String)isTapeOn_B + " C:" + (String)isTapeOn_C + " D:" + (String)isTapeOn_D + " E:" + (String)isTapeOn_E;
    //Serial.println(state);
    TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  }
}

void CheckGlobalStates(void){
  //Serial.print("STATE: ");
  //Serial.println(state);
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
    case FOUND_T_LINE:
      RespFoundTLine();
      break;
    case AT_BACK_OF_BOX:
      RespAtBackOfBox();
      break;
    case CROSSED_ONE_LINE:
      RespCountedOneLine();
      break;
    case CROSSED_TWO_LINES:
      RespCountedTwoLines();
      break;
    case CROSSED_THREE_LINES:
      RespCountedThreeLines();
      break;
    case PAUSE_AT_LINE:
      RespPauseAtLine();
      break;
    case ALIGN_TO_SHOOT:
      RespAlignToShoot();
      break;
    case PIVOT_SHOOT:
      RespPivotShoot();
      break;
    case ACTUALLY_SHOOT:
      RespActuallyShoot();
      break;
    case ALIGN_AFTER_PIVOT:
      RespAlignAfterPivot();
      break;
    case DONE:
      RespDone();
      break;
    case SHOOT:
      RespShoot();
      break;
    case BLOCK:
      RespBlock();
      break;
    case YOLO:
      RespYolo();
      break;
    default:
      break;
  }
  
}

void RespYolo(){
  delay(1000);
  makeMotorsStop();
}

void findFreqComms(){
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), freqCountComms, FALLING);
  prev_time_comms = micros();
}

void freqCountComms(){
  attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
  pwm_value_comms = micros() - prev_time_comms;
}

int decodeSignalFromComms(){
  int commsState = map(pwm_value_comms, 0, 1920, 0, 11);
  /*
   * 0 - gyro negative
   * 1 - gyro positive
   */
//   Serial.print(pwm_value_comms);
//   Serial.print(" ");
////   Serial.print(state);
  return commsState;
}


void RespDone(){
   makeMotorsStop();
}

void RespAlignToShoot(){
  // if we read b, c, & d, we are good
if(ReadTapeSensor_B_1() && ReadTapeSensor_B_2() && ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_D_2()){
     // WE CAN SHOOT
//    digitalWrite(PIN_RED_DEBUG, HIGH);
//    digitalWrite(PIN_GREEN_DEBUG, HIGH);
//    digitalWrite(PIN_YELLOW_DEBUG, HIGH);
//    state = PIVOT_SHOOT;
    pivot_CC = true;
    makeMotorsStop(); 
    pivot_timer = millis();
  }
/*  else if(ReadTapeSensor_C() && ReadTapeSensor_B_1() && !ReadTapeSensor_D_1()){
  digitalWrite(PIN_YELLOW_DEBUG, LOW);
   digitalWrite(PIN_GREEN_DEBUG, LOW);
    digitalWrite(PIN_RED_DEBUG, HIGH);
  } 
  else if (ReadTapeSensor_C() && !ReadTapeSensor_B_1() && ReadTapeSensor_D_1()){
  digitalWrite(PIN_YELLOW_DEBUG, LOW);
   digitalWrite(PIN_GREEN_DEBUG, LOW);
    digitalWrite(PIN_RED_DEBUG, HIGH);
  }*/
  else{ // if(ReadTapeSensor_C() && !ReadTapeSensor_B_1() && !ReadTapeSensor_D_1){
     // NOT ALIGNED

     if(alignLeft){
        // trying to turn counter clockwise
        makeMotorsSpinCC();  
//        digitalWrite(PIN_RED_DEBUG, LOW);
//        digitalWrite(PIN_YELLOW_DEBUG, LOW);
//        digitalWrite(PIN_GREEN_DEBUG, LOW);      
        if (millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
          // turn the other way
          alignLeft = false;
          fake_timer = millis();
        } 
     } else {
        makeMotorsSpinCL();
//        digitalWrite(PIN_GREEN_DEBUG, LOW);
//        digitalWrite(PIN_YELLOW_DEBUG, LOW);
//        digitalWrite(PIN_RED_DEBUG, HIGH);
        if (millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
          // turn the other way
          // WE ARE FUCKED IF STILL CAN'T FIND
          alignLeft = true;
          fake_timer = millis();
          //Serial.println("FUCKED");
        } 
     }
  }
}



void RespCountedOneLine(){
   // keep going forward 
  // digitalWrite(PIN_GREEN_DEBUG, HIGH);
    
   if(millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
      // fake timer there to make sure that 
     // we aren't reading the same line 
     if(ReadTapeSensor_B_1()){
//      digitalWrite(PIN_GREEN_DEBUG, HIGH);
//      digitalWrite(PIN_YELLOW_DEBUG, HIGH);
        state=SHOOT;
        //change back 
//       state = ALIGN_TO_SHOOT;
       fake_timer = millis();
       numLinesCounted = 2;
       makeMotorsStop();
     }
   }
}

void RespShoot(){
  //change this shit right here
  fake_timer=millis();
   makeMotorsStop(); 

  if(shoot_count < 3){
    respondToShoot();
    Serial.print("shooting ");
    Serial.println(shoot_count);
  }
  else{
    Serial.println("wtfffff");
    // DONE SHOOTING
    shoot_count = 0;
    endShoot();
    delay(1000);
    makeMotorSpeedFast();
    makeMotorsMoveForward();
//   digitalWrite(PIN_RED_DEBUG, HIGH);
//    digitalWrite(PIN_GREEN_DEBUG, HIGH);
//    digitalWrite(PIN_YELLOW_DEBUG, HIGH);
    if (numLinesCounted<=3){
      Serial.println("HIT!!! \n\n\n\n\n");
      state=PAUSE_AT_LINE;
      analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_FORWARD);
      delay(100);
      fake_timer=millis();
    } else { 
      state=BLOCK;
      fake_timer = millis();
      makeMotorsMoveRight();
    }
  }
}

void RespCountedTwoLines(){
   if(millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
      // fake timer there to make sure that 
     // we aren't reading the same line 
     if(ReadTapeSensor_B_1()){
      state=SHOOT;
      //changed thiss
//       state = ALIGN_TO_SHOOT;
       numLinesCounted = 3;
       makeMotorsStop();
       fake_timer = millis();
     }
   }
}

void RespCountedThreeLines(){
  makeMotorsMoveRight();
  /*
  if(millis() = fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
      // fake timer there to make sure that 
     // we aren't reading the same line 
     // check if read line
     if(ReadTapeSensor_C()){
       state = PAUSE_AT_LINE;
       fake_timer = millis();
       numLinesCounted = 3;
       makeMotorsStop();
     }
   }*/
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
  if(startGyroNegative && (comm_ret == 0)){
    analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CC);
//    digitalWrite(PIN_GREEN_DEBUG, LOW);
    //makeMotorsSpinCL();
  } else if(!startGyroNegative && (comm_ret == 1)){
    analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CL);
//    digitalWrite(PIN_YELLOW_DEBUG, HIGH);
    //makeMotorsSpinCC();
  } else{
//    digitalWrite(PIN_YELLOW_DEBUG, LOW);
    state = ORIENTATION_STRAIGHT;
    fake_timer = millis();
  }
}

void RespOrientationStraight(){
  // stop
  //analogWrite(PIN_OUTPUT_SPEED_CONTROL, 60);
  makeMotorSpeedFast();

  makeMotorsJiggleBackwards();
 
  // go backward for 2 seconds
  if( millis() - fake_timer >= 2000){
    // go backward
    state = AT_BACK_OF_BOX;
    fake_timer = millis();
  }
}

void RespAtBackOfBox(){
    makeMotorSpeedNormal();
    makeMotorsMoveForward();
    //was tape sensor c
    if (ReadTapeSensor_B_1()){
      state = PAUSE_AT_LINE;
      numLinesCounted = 1;
      fake_timer = millis();
    }
}

void RespPauseAtLine(){
//  digitalWrite(PIN_RED_DEBUG, HIGH);
//    digitalWrite(PIN_GREEN_DEBUG, LOW);
//    digitalWrite(PIN_YELLOW_DEBUG, HIGH);
  if(millis()-fake_timer > TIME_TO_PAUSE){
      // we have waited long enough
      if(numLinesCounted == 2){ // || numLinesCounted == 3){
        Serial.println("why aren't you movoing");
         state = CROSSED_TWO_LINES;
//         makeMotorSpeedNormal();
//         makeMotorsMoveForward();
         fake_timer = millis(); 
      }
      else if(numLinesCounted == 1){ //|| numLinesCounted == 2){
         state = CROSSED_ONE_LINE;
         
         makeMotorSpeedNormal();      
        makeMotorsMoveForward();
        fake_timer = millis();   
      }
      else if(numLinesCounted == 3){
         state = CROSSED_THREE_LINES;
         
         makeMotorSpeedNormal();
         makeMotorsMoveForward();
         fake_timer = millis(); 
      }
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
    Serial.print("Subtracted 1: ");
    Serial.println(NUM_PULSES);
  } else if (count == CALIBRATE_COUNT && !calibrate_tick) {
    calibrate_tick = true;
    count = 1;
    NUM_PULSES++;
    Serial.print("Added 1: ");
    Serial.println(NUM_PULSES);
  }
  Serial.print("Count: ");
  Serial.println(count);
  Pulse(NUM_PULSES); 
  pulses_done = false;
  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
}

void endShoot(){
  digitalWrite(FLYWHEEL_ON, LOW);
  EndPulse();
}

void respondToShoot() {
    digitalWrite(FLYWHEEL_ON, HIGH);
    if(IsPulseFinished()){
      pulses_done = true;
    }
    if(TMRArd_IsTimerExpired(STEPPER_WAIT_TIMER)){
      setupPulses();
    }
}



void RespBlock(){
  if(millis()-fake_timer > TIME_MOVE_RIGHT){
    state = DONE;
    makeMotorsStop();
  }
  
}



void RespFoundTLine(){
  makeMotorsStop();
}

void RespLeftEndLine(){
  // won't get here right now
  if(isTapeOn_B && isTapeOn_C && isTapeOn_D){
    makeMotorsStop();
    state = FOUND_T_LINE;
  }
}

void RespRightEndLine(){
  if(isTapeOn_B && isTapeOn_C && isTapeOn_D){
    // now drive left
    makeMotorsMoveLeft();
    state = FOUND_T_LINE;
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
  pinMode(PIN_YELLOW_DEBUG, OUTPUT);
  pinMode(PIN_RED2_DEBUG, OUTPUT);
  
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  pinMode(PIN_OUTPUT_SPEED_CONTROL, OUTPUT);
  pinMode(PIN_INPUT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_GAME_START, INPUT_PULLUP);

  pinMode(PULSE_STEPPER, OUTPUT);
  pinMode(FLYWHEEL_ON, OUTPUT);

}

/* ====================================================================
 *  UpdateTapeSensorVars function
 *      updates the tape sensor globals
 * ====================================================================
 */

void UpdateTapeSensorVars(void){
   // Change state based on tape sensors
   if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
      //state = STATE_ON_CROSS_LINE_A_B_C_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 31
   }  
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // Read B, C, D, E
      //state = STATE_BOTTOM_T_LINE_B_C_D_E;
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 15   
   }
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // Read B, C, D
//      state = STATE_ON_HORZ_LINE_B_C_D; 
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 14  
   }  


   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
//      state = STATE_NO_TAPE_DETECTED;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 0
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
//      state = STATE_ONLY_TAPE_E;
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 1   
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // right edge
//      state = STATE_ONLY_TAPE_D;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 2
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // back right edge, mirrors B E
//      state = STATE_BACK_RIGHT_ON_LINE_D_E;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 3
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
//      state = STATE_ONLY_TAPE_C;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 4
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE C and E
      // back center on 
//      state = STATE_BACK_CENTER_LINE_C_E;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 5
   }     
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE C and D
      // center right on 
//      state = STATE_CENTER_RIGHT_C_D;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 6
   }     
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE C and D and E
      // back right corner  
//      state = STATE_BACK_RIGHT_CORNER_C_E_D;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 7
   }     
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE B
      // left edge  
//      state = STATE_ONLY_TAPE_B;   
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 8
   }     
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE B and E
      // left diagonal, mirrors D and E
//      state = STATE_BACK_RIGHT_ON_LINE_B_E;   
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 9
   }     
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE B and D
      // could be on a corner
//      state = STATE_COULD_BE_ON_CORNER_B_D;   
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 10
   }     
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE B and D and E
      // could be on a corner
//      state = STATE_ONLY_TAPE_B_D_E;   
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 11
   }     
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE B and C
      // on left side of line, mirrors D and C
//      state = STATE_CENTER_LEFT_B_C;   
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 12
   }     
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE B and C and E
      // on left side of line, mirrors D and C E
//      state = STATE_BACK_LEFT_CORNER_B_C_E;   
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 13
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A
      // recognize just center top
//      state = STATE_ONLY_TAPE_A;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 16
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE A and E
      // just top and bottom, like B and D
//      state = STATE_COULD_BE_ON_CORNER_A_E;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 17
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and D
      // similar to A and B
//      state = STATE_TOP_RIGHT_CORNER_A_D;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 18
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE A and D and E
//      state = STATE_ONLY_TAPE_A_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 19
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and C
      // similar to C E 
//      state = STATE_TOP_CENTER_LINE_A_C;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 20
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE A and C and E
//      state = STATE_TOP_CENTER_LINE_A_C_E;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 21
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and C and D
//      state = STATE_TOP_RIGHT_CORNER_A_C_D;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 22
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE A and C and D and E
      // mirrors ACBE
//      state = STATE_RIGHT_SIDE_A_C_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 23
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and B
      // mirrros A D
//      state = STATE_TOP_LEFT_DIAG_A_B;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 24
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE A and B and E
//      state = STATE_LEFT_SIDE_A_B_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 25
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and B and D
//      state = STATE_TOP_SIDE_A_B_D;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 26
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and B and D and E
//      state = STATE_ALL_BUT_CENTER_A_B_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 27
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and B and C
//      state = STATE_TOP_LEFT_CORNER_A_B_C;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 28
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D_1() && ReadTapeSensor_E()){
      // READ TAPE A and B and C and E
//      state = STATE_LEFT_SIDE_A_B_C_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 29
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B_1() && 
       ReadTapeSensor_C() && ReadTapeSensor_D_1() && !ReadTapeSensor_E()){
      // READ TAPE A and B and C and d
//      state = STATE_TOP_T_LINE_A_B_C_D;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 30
   }     
}

void RespAlignAfterPivot(){
  // before, we turned for TIME_TO_PIVOT amount of time CL
  if(millis() - pivot_timer >= TIME_TO_PIVOT){
    // stop
    makeMotorsStop();
    // now move on
    state = PAUSE_AT_LINE;
    fake_timer = millis();
  }
  
}

void RespActuallyShoot(){
  makeMotorsStop();
  // SHOOT HERE
  if (shots_fired == 1){
    pivot_timer = millis();
    state = PIVOT_SHOOT;
//    digitalWrite(PIN_RED_DEBUG, HIGH);
//    digitalWrite(PIN_GREEN_DEBUG, HIGH);
//    digitalWrite(PIN_YELLOW_DEBUG, HIGH);
  } else {
    // WE HAVE SHOT BOTH OF THEM NOW MOVE ON.
    // we have to turn ourselves around again
    state = ALIGN_AFTER_PIVOT;
    pivot_timer = millis();
    makeMotorSpeedNormal();
    makeMotorsSpinCC(); // to un-pivot
//    digitalWrite(PIN_RED_DEBUG, HIGH);
//    digitalWrite(PIN_GREEN_DEBUG, LOW);
//    digitalWrite(PIN_YELLOW_DEBUG, HIGH);
  }
}

void  RespPivotShoot(){
   // motors stopped from align to shoot
   // turn tiny bit left
   if (pivot_CC){
//    digitalWrite(PIN_RED_DEBUG, LOW);
//    digitalWrite(PIN_GREEN_DEBUG, HIGH);
//    digitalWrite(PIN_YELLOW_DEBUG, LOW);
    makeMotorSpeedFast();
     makeMotorsSpinCC();
     if(millis() - pivot_timer > TIME_TO_PIVOT){
        // actually shoot
        pivot_CC = false;
        state = ACTUALLY_SHOOT;
        shots_fired = 1;
        makeMotorsStop();
     }
   } else {
//    digitalWrite(PIN_RED_DEBUG, HIGH);
//    digitalWrite(PIN_GREEN_DEBUG, LOW);
//    digitalWrite(PIN_YELLOW_DEBUG, LOW);
    makeMotorSpeedFast();
     makeMotorsSpinCL();
     if(millis() - pivot_timer > TIME_TO_PIVOT * 2){
        // actually shoot
        // times 2 because have to unrotate ourselves
        pivot_CC = false;
        state = ACTUALLY_SHOOT;
        shots_fired = 2;
        makeMotorsStop();
     }
   }
   // turn tiny bit right
}
