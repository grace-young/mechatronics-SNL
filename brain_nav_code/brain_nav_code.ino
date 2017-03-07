#include <Timers.h>

#define PIN_SENSOR_A 7
#define PIN_SENSOR_B_1 6
#define PIN_SENSOR_C 4
#define PIN_SENSOR_D_1 8
#define PIN_SENSOR_E 5

// need pins for these:
#define PIN_SENSOR_B_2 NUMBER GOES HERE
#define PIN_SENSOR_B_2 NUMBER GOES HERE


#define PIN_COMMS_IN_GYRO        2
#define PIN_OUTPUT_MOTOR_CONTROL 3
#define PIN_OUTPUT_SPEED_CONTROL 9
#define PIN_INPUT_BUMPER A0

#define PIN_GAME_START           10

#define TIME_INTERVAL      500
#define JIGGLE_INTERVAL    400
#define TIME_INTERVAL_2    1000
#define TIME_BETWEEN_LINE_TAPE_READS  250


#define TIMER_0            0

#define COMM_MOTOR_NORMAL  50
#define COMM_MOTOR_SLOW    100
#define COMM_MOTOR_FAST    200
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


static bool startGyroNegative = false;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;

static int numLinesCounted = 0;

typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT,
  FOUND_T_LINE, AT_BACK_OF_BOX, CROSSED_ONE_LINE, CROSSED_TWO_LINES, 
  CROSSED_THREE_LINES, PAUSE_AT_LINE, ALIGN_TO_SHOOT, DONE
} States_t;

States_t state;
unsigned char isTapeOn_A;
unsigned char isTapeOn_B;
unsigned char isTapeOn_C;
unsigned char isTapeOn_D;
unsigned char isTapeOn_E;

static bool alignLeft;

void setup() {
  Serial.begin(19200);
  //TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  isTapeOn_A = false;
  isTapeOn_B = false;
  isTapeOn_C = false;
  isTapeOn_D = false;
  isTapeOn_E = false;
  state = WAIT;
  
  makeMotorsMoveForward();

  SetupPins();
  
  alignLeft=true;
  
  TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 

  //Interupt for reading the Comms from gyro
   attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
   fake_timer = millis();
}


void loop() {
  makeMotorsFast();
  UpdateTapeSensorVars();
  // we know the tape sensor values are right noe
  CheckGlobalStates();
  //Serial.println(state);
  if(TMRArd_IsTimerExpired(TIMER_0)){
    String toprint = "state: " + (String)state + "\nA: " + (String)isTapeOn_A + " B:"+(String)isTapeOn_B + " C:" + (String)isTapeOn_C + " D:" + (String)isTapeOn_D + " E:" + (String)isTapeOn_E;
    //Serial.println(state);
    TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  }
}

void CheckGlobalStates(void){
  Serial.print("STATE: ");
  Serial.println(state);
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
    case DONE:
      RespDone();
    default:
      break;
  }
  
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
   Serial.print(pwm_value_comms);
   Serial.print(" ");
   Serial.print(state);
   Serial.print(" ");
   Serial.println(commsState);
  return commsState;
}

void RespPauseAtLine(){
  if(millis()-fake_timer > 1000){
      // we have waited long enough
      if(numLinesCounted == 2){
         state = CROSSED_TWO_LINES;
         fake_timer = millis(); 
         makeMotorsMoveForward();
      }
      else if(numLinesCounted == 1){
         state = CROSSED_ONE_LINE;
         fake_timer = millis();         
        makeMotorsMoveForward();
      }
      else if(numLinesCounted == 3){
         state = CROSSED_THREE_LINES;
         fake_timer = millis(); 
         makeMotorsStop();
      }
   } 
}

void RespDone(){
   makeMotorsStop(); 
}

void RespAlignToShoot(){
  // if we read b, c, & d, we are good
  if(ReadTapeSensor_B() && ReadTapeSensor_C() && ReadTapeSensor_D()){
     // WE CAN SHOOT
    Serial.println("GOING TO DONE");
    state = DONE;
    makeMotorsStop(); 
  }
  else if(ReadTapeSensor_C() && ReadTapeSensor_B() && !ReadTapeSensor_D()){
    Serial.println("read C & B & not D");
  } 
  else if (ReadTapeSensor_C() && !ReadTapeSensor_B() && ReadTapeSensor_D()){
    Serial.println("read C & D & not B");
  }else if(ReadTapeSensor_C() && !ReadTapeSensor_B() && !ReadTapeSensor_D){
     // NOT ALIGNED
     if(alignLeft){
        // trying to turn counter clockwise
        makeMotorsSpinCC();        
        Serial.println("spinning counter clockwise");
        if (millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
          // turn the other way
          Serial.println("tried to align left");
          alignLeft = false;
          fake_timer = millis();
        } 
     } else {
        makeMotorsSpinCL();
        Serial.println("spinning clockwise");
        if (millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
          // turn the other way
          // WE ARE FUCKED IF STILL CAN'T FIND
          alignLeft = true;
          fake_timer = millis();
          Serial.println("FUCKED");
        } 
     }
  }
  
}

void RespCountedOneLine(){
   // keep going forward 
   if(millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
      // fake timer there to make sure that 
     // we aren't reading the same line 
     if(ReadTapeSensor_C()){
       state = ALIGN_TO_SHOOT;
       fake_timer = millis();
       numLinesCounted = 2;
       makeMotorsStop();
     }
   }
}

void RespCountedTwoLines(){
   if(millis() - fake_timer > TIME_BETWEEN_LINE_TAPE_READS){
      // fake timer there to make sure that 
     // we aren't reading the same line 
     if(ReadTapeSensor_C()){
       state = PAUSE_AT_LINE;
       fake_timer = millis();
       numLinesCounted = 2;
       makeMotorsStop();
     }
   }
}

void RespCountedThreeLines(){
  makeMotorsStop();
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

void RespAtBackOfBox(){
    makeMotorsMoveForward();
    if (ReadTapeSensor_C()){
      state = CROSSED_ONE_LINE;
      fake_timer = millis();
      numLinesCounted = 1;
    }
}

void RespToWait(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL,COMM_MOTOR_COAST_STOP);
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
  
  analogWrite(PIN_OUTPUT_SPEED_CONTROL, COMM_MOTOR_SLOW);
  if(startGyroNegative && (decodeSignalFromComms() == 0)){
    analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CL);
  } else if(!startGyroNegative && (decodeSignalFromComms() == 1)){
    analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CC);
  } else{
    state = ORIENTATION_STRAIGHT;
    fake_timer = millis();
  }
}

void RespOrientationStraight(){
  // stop
  makeMotorsMoveBackward();
  // go backward for 2 seconds
  if( millis() - fake_timer >= 2000){
    // go backward
    state = AT_BACK_OF_BOX;
    fake_timer = millis();
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
    return !digitalRead(PIN_SENSOR_A);
}

bool ReadTapeSensor_B_1(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_B_1);
}

bool ReadTapeSensor_B_2(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_B_2);
}

bool ReadTapeSensor_C(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_C);
}

bool ReadTapeSensor_D_1(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_D_1);
}

bool ReadTapeSensor_D_2(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_D_2);
}

bool ReadTapeSensor_E(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_E);
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
 
void makeMotorsSlow(){
  analogWrite(PIN_OUTPUT_SPEED_CONTROL, COMM_MOTOR_SLOW); 
}

void makeMotorsFast(){
  analogWrite(PIN_OUTPUT_SPEED_CONTROL,COMM_MOTOR_FAST);
}

void makeMotorsStop(){
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_STOP);
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
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  pinMode(PIN_OUTPUT_SPEED_CONTROL, OUTPUT);
  pinMode(PIN_INPUT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_GAME_START, INPUT_PULLUP);
}

/* ====================================================================
 *  UpdateTapeSensorVars function
 *      updates the tape sensor globals
 * ====================================================================
 */

void UpdateTapeSensorVars(void){
   // Change state based on tape sensors
   if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
      //state = STATE_ON_CROSS_LINE_A_B_C_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 31
   }  
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
      // Read B, C, D, E
      //state = STATE_BOTTOM_T_LINE_B_C_D_E;
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 15   
   }
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // Read B, C, D
//      state = STATE_ON_HORZ_LINE_B_C_D; 
        isTapeOn_A = false;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 14  
   }  


   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
//      state = STATE_NO_TAPE_DETECTED;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 0
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
//      state = STATE_ONLY_TAPE_E;
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 1   
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // right edge
//      state = STATE_ONLY_TAPE_D;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 2
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
      // back right edge, mirrors B E
//      state = STATE_BACK_RIGHT_ON_LINE_D_E;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 3
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // All tapes sensors are READ TAPE
//      state = STATE_ONLY_TAPE_C;   
        isTapeOn_A = false;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 4
   }  
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (!ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
      // READ TAPE A and D and E
//      state = STATE_ONLY_TAPE_A_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = true;
      // 19
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
      // READ TAPE A and C and E
//      state = STATE_TOP_CENTER_LINE_A_C_E;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 21
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // READ TAPE A and C and D
//      state = STATE_TOP_RIGHT_CORNER_A_C_D;   
        isTapeOn_A = true;
        isTapeOn_B = false;
        isTapeOn_C = true;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 22
   }     
   else if (ReadTapeSensor_A() && !ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
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
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
      // READ TAPE A and B and E
//      state = STATE_LEFT_SIDE_A_B_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 25
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // READ TAPE A and B and D
//      state = STATE_TOP_SIDE_A_B_D;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 26
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // READ TAPE A and B and D and E
//      state = STATE_ALL_BUT_CENTER_A_B_D_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = true;
        isTapeOn_E = false;
      // 27
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       !ReadTapeSensor_C() && !ReadTapeSensor_D() && !ReadTapeSensor_E()){
      // READ TAPE A and B and C
//      state = STATE_TOP_LEFT_CORNER_A_B_C;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = false;
        isTapeOn_D = false;
        isTapeOn_E = false;
      // 28
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && !ReadTapeSensor_D() && ReadTapeSensor_E()){
      // READ TAPE A and B and C and E
//      state = STATE_LEFT_SIDE_A_B_C_E;   
        isTapeOn_A = true;
        isTapeOn_B = true;
        isTapeOn_C = true;
        isTapeOn_D = false;
        isTapeOn_E = true;
      // 29
   }     
   else if (ReadTapeSensor_A() && ReadTapeSensor_B() && 
       ReadTapeSensor_C() && ReadTapeSensor_D() && !ReadTapeSensor_E()){
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

