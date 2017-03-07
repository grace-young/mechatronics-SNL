#include <Timers.h>

#define SENSOR_A 7
#define SENSOR_B 6
#define SENSOR_C 4
#define SENSOR_D 8
#define SENSOR_E 5

#define PIN_COMMS_IN_GYRO        2
#define PIN_OUTPUT_MOTOR_CONTROL 3
#define PIN_OUTPUT_SPEED_CONTROL 9
#define PIN_INPUT_BUMPER A0

#define PIN_GAME_START           10

#define TIME_INTERVAL      500
#define JIGGLE_INTERVAL    400
#define TIME_INTERVAL_2    1000

#define TIMER_0            0

#define COMM_MOTOR_NORMAL  50
#define COMM_MOTOR_SLOW    100
#define COMM_MOTOR_FAST    200

#define COMM_MOTOR_COAST_STOP 20
#define COMM_MOTOR_FORWARD 40
#define COMM_MOTOR_BACKWARD 60
#define COMM_MOTOR_LEFT 80
#define COMM_MOTOR_RIGHT 100
#define COMM_MOTOR_SPIN_CC 120
#define COMM_MOTOR_SPIN_CL 140
#define COMM_MOTOR_STOP 160



#define slow 80
#define normal 160
#define fast 240

static bool startGyroNegative = false;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;


/*STATE_ON_CROSS_LINE_A_B_C_D_E, STATE_BOTTOM_T_LINE_B_C_D_E, STATE_ON_HORZ_LINE_B_C_D,  
  STATE_NO_TAPE_DETECTED, STATE_ONLY_TAPE_E, STATE_ONLY_TAPE_D,
  STATE_BACK_RIGHT_ON_LINE_D_E, STATE_ONLY_TAPE_C, STATE_BACK_CENTER_LINE_C_E,   
  STATE_CENTER_RIGHT_C_D, STATE_BACK_RIGHT_CORNER_C_E_D, STATE_ONLY_TAPE_B,   
  STATE_BACK_RIGHT_ON_LINE_B_E, STATE_COULD_BE_ON_CORNER_B_D,   
  STATE_ONLY_TAPE_B_D_E, STATE_CENTER_LEFT_B_C, STATE_BACK_LEFT_CORNER_B_C_E,  
  STATE_ONLY_TAPE_A, STATE_COULD_BE_ON_CORNER_A_E, STATE_TOP_RIGHT_CORNER_A_D,   
  STATE_ONLY_TAPE_A_D_E, STATE_TOP_CENTER_LINE_A_C, STATE_TOP_CENTER_LINE_A_C_E, 
  STATE_TOP_RIGHT_CORNER_A_C_D, STATE_RIGHT_SIDE_A_C_D_E, STATE_TOP_LEFT_DIAG_A_B,   
  STATE_LEFT_SIDE_A_B_E, STATE_TOP_SIDE_A_B_D, STATE_ALL_BUT_CENTER_A_B_D_E,   
  STATE_TOP_LEFT_CORNER_A_B_C, STATE_LEFT_SIDE_A_B_C_E, STATE_TOP_T_LINE_A_B_C_D
 */

typedef enum {
  WAIT, START, ORIENTATION_STRAIGHT, LOOKING_FOR_TAPE, FOUND_HORZ_LINE, RIGHT_END_OF_LINE, LEFT_END_OF_LINE,
  FOUND_T_LINE, AT_BACK_OF_BOX
} States_t;

States_t state;
unsigned char isTapeOn_A;
unsigned char isTapeOn_B;
unsigned char isTapeOn_C;
unsigned char isTapeOn_D;
unsigned char isTapeOn_E;

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
  
  TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 

  //Interupt for reading the Comms from gyro
   attachInterrupt(digitalPinToInterrupt(PIN_COMMS_IN_GYRO), findFreqComms, RISING);
   fake_timer = millis();
}


void loop() {
  makeMotorsFast();
  //makeMotorsSlow();
  UpdateTapeSensorVars();
  // we know the tape sensor values are right noe
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, 40);
  Serial.println("before delay");
  delay(1000);
  Serial.println("after delay");
  while(digitalRead(SENSOR_E)){
    analogWrite(PIN_OUTPUT_MOTOR_CONTROL, 60);
    Serial.println("backwards");
  }
  analogWrite(PIN_OUTPUT_MOTOR_CONTROL, 160);
  delay(1000000);
  CheckGlobalStates();
  //Serial.println(state);
  if(TMRArd_IsTimerExpired(TIMER_0)){
    String toprint = "state: " + (String)state + "\nA: " + (String)isTapeOn_A + " B:"+(String)isTapeOn_B + " C:" + (String)isTapeOn_C + " D:" + (String)isTapeOn_D + " E:" + (String)isTapeOn_E;
    //Serial.println(state);
    TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  }
  //if(goingFirstShootSpot){
  //  goToFirstShootingSpot();
  //}
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
    case LOOKING_FOR_TAPE:
      RespStateLookingForTape();
      break;
    case FOUND_HORZ_LINE:
      RespFoundHorzLine();
      break;
    case RIGHT_END_OF_LINE:
      RespRightEndLine();
      break;
    case LEFT_END_OF_LINE:
      RespLeftEndLine();
      break;
    case FOUND_T_LINE:
      RespFoundTLine();
      break;
    case AT_BACK_OF_BOX:
      RespAtBackOfBox();
      break;
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

void RespAtBackOfBox(){
    makeMotorsMoveForward();
    if(millis() - fake_timer >= 1000){
      makeMotorsStop();
    }
    if (ReadTapeSensor_A()){
      
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

void RespStateLookingForTape(){
  if(isTapeOn_A && isTapeOn_B && isTapeOn_C && isTapeOn_D){
    // FOUND A T
    makeMotorsStop();
    state = FOUND_T_LINE;
  }
  else if (isTapeOn_B && isTapeOn_C && isTapeOn_D){
    // NOW ON A HORZ LINE
    makeMotorsMoveRight();
    state = FOUND_HORZ_LINE;
  }
}

void RespFoundHorzLine(){
  // right now this is assuming it's aligned exactly straight to tape
  // so need to fix edge cases later   
  if(isTapeOn_A && isTapeOn_B && isTapeOn_C && isTapeOn_D){
    makeMotorsMoveForward();
    state = FOUND_T_LINE;
  } else if(isTapeOn_B && isTapeOn_C){
    makeMotorsMoveLeft();
    state = RIGHT_END_OF_LINE;
  } else if(isTapeOn_C && isTapeOn_D){
    makeMotorsMoveRight();
    state = LEFT_END_OF_LINE;
  }
}

bool ReadTapeSensor_A(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(SENSOR_A);
}

bool ReadTapeSensor_B(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(SENSOR_B);
}

bool ReadTapeSensor_C(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(SENSOR_C);
}

bool ReadTapeSensor_D(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(SENSOR_D);
}

bool ReadTapeSensor_E(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(SENSOR_E);
}

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
/*
void RespToStates(void){
  if(TMRArd_IsTimerExpired(TIMER_0)){
    Serial.println(ReadTapeSensor_A());
    Serial.println(ReadTapeSensor_B());
    Serial.println(ReadTapeSensor_C());
    Serial.println(ReadTapeSensor_D());
    Serial.println(ReadTapeSensor_E());
    switch(state){
      case STATE_ON_CROSS_LINE_A_B_C_D_E:  
        Serial.println("STATE_ON_CROSS_LINE_A_B_C_D_E");
        break;
      case STATE_BOTTOM_T_LINE_B_C_D_E:
        Serial.println("STATE_BOTTOM_T_LINE_B_C_D_E");
        break;
      case STATE_ON_HORZ_LINE_B_C_D:
        Serial.println("STATE_ON_HORZ_LINE_B_C_D");
        break;
      case STATE_NO_TAPE_DETECTED:
        Serial.println("STATE_NO_TAPE_DETECTED");
        break;
      case STATE_ONLY_TAPE_E:
        Serial.println("STATE_ONLY_TAPE_E");
        break;
      case STATE_ONLY_TAPE_D:   
        Serial.println("STATE_ONLY_TAPE_D");
        break;
      case STATE_BACK_RIGHT_ON_LINE_D_E:   
        Serial.println("STATE_BACK_RIGHT_ON_LINE_D_E");
        break;      
      case STATE_ONLY_TAPE_C:   
        Serial.println("STATE_ONLY_TAPE_C");
        break;            
      case STATE_BACK_CENTER_LINE_C_E:   
        Serial.println("STATE_BACK_CENTER_LINE_C_E");
        break;                  
      case STATE_CENTER_RIGHT_C_D:   
        Serial.println("STATE_CENTER_RIGHT_C_D");
        break;                       
      case STATE_BACK_RIGHT_CORNER_C_E_D:   
        Serial.println("STATE_BACK_RIGHT_CORNER_C_E_D");
        break;                       
      case STATE_ONLY_TAPE_B:   
        Serial.println("STATE_ONLY_TAPE_B");
        break;                      
      case STATE_BACK_RIGHT_ON_LINE_B_E:   
        Serial.println("STATE_BACK_RIGHT_ON_LINE_B_E");
        break;      
      case STATE_COULD_BE_ON_CORNER_B_D:   
        Serial.println("STATE_COULD_BE_ON_CORNER_B_D");
        break;      
      case STATE_ONLY_TAPE_B_D_E:   
        Serial.println("STATE_ONLY_TAPE_B_D_E");
        break;      
      case STATE_CENTER_LEFT_B_C:   
        Serial.println("STATE_CENTER_LEFT_B_C");
        break;            
      case STATE_BACK_LEFT_CORNER_B_C_E:   
        Serial.println("STATE_BACK_LEFT_CORNER_B_C_E");
        break;                  
      case STATE_ONLY_TAPE_A:   
        Serial.println("STATE_ONLY_TAPE_A");
        break;                        
      case STATE_COULD_BE_ON_CORNER_A_E:   
        Serial.println("STATE_COULD_BE_ON_CORNER_A_E");
        break;                          
      case STATE_TOP_RIGHT_CORNER_A_D:   
        Serial.println("STATE_TOP_RIGHT_CORNER_A_D");
        break;                                
      case STATE_ONLY_TAPE_A_D_E:   
        Serial.println("STATE_ONLY_TAPE_A_D_E");
        break;                                
      case STATE_TOP_CENTER_LINE_A_C:   
        Serial.println("STATE_TOP_CENTER_LINE_A_C");
        break;                                      
      case STATE_TOP_CENTER_LINE_A_C_E:   
        Serial.println("STATE_TOP_CENTER_LINE_A_C_E");
        break;                                            
      case STATE_TOP_RIGHT_CORNER_A_C_D:   
        Serial.println("STATE_TOP_RIGHT_CORNER_A_C_D");
        break;                                                  
      case STATE_RIGHT_SIDE_A_C_D_E:   
        Serial.println("STATE_RIGHT_SIDE_A_C_D_E");
        break;                                                  
      case STATE_TOP_LEFT_DIAG_A_B:   
        Serial.println("STATE_TOP_LEFT_DIAG_A_B");
        break;                                                        
      case STATE_LEFT_SIDE_A_B_E:   
        Serial.println("STATE_LEFT_SIDE_A_B_E");
        break;                                                              
      case STATE_TOP_SIDE_A_B_D:   
        Serial.println("STATE_TOP_SIDE_A_B_D");
        break;                                                                    
      case STATE_ALL_BUT_CENTER_A_B_D_E:   
        Serial.println("STATE_ALL_BUT_CENTER_A_B_D_E");
        break;                           
      case STATE_TOP_LEFT_CORNER_A_B_C:   
        Serial.println("STATE_TOP_LEFT_CORNER_A_B_C");
        break; 
      case STATE_LEFT_SIDE_A_B_C_E:   
        Serial.println("STATE_LEFT_SIDE_A_B_C_E");
        break;       
      case STATE_TOP_T_LINE_A_B_C_D:   
        Serial.println("STATE_TOP_T_LINE_A_B_C_D");
        break;       
    }
    TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  }
}

*/

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

void SetupPins(){
  pinMode(SENSOR_A, INPUT);
  pinMode(SENSOR_B, INPUT);
  pinMode(SENSOR_C, INPUT);
  pinMode(SENSOR_D, INPUT);
  pinMode(SENSOR_E, INPUT);
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  pinMode(PIN_OUTPUT_SPEED_CONTROL, OUTPUT);
  pinMode(PIN_INPUT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_GAME_START, INPUT_PULLUP);
}

/*
void goToFirstShootingSpot(){
  switch (stateNumber) {
    case FWD_SAFE_SPACE:
         if (digitalRead(SENSOR_C)) {
            analogWrite(motorStateOut, forward);
         } else {
            stateNumber = ALIGN_SAFE_SPACE;
            TMRArd_InitTimer(TIMER_0, JIGGLE_INTERVAL); 
         }
         break;
    case ALIGN_SAFE_SPACE:
         if (digitalRead(SENSOR_B)) {
            
         }
         break:
    case MOVE_ON_LINE:

         break:
    default:

         break:
  }
  
//  if(!Cseen1){
//    analogWrite(motorStateOut, forward);
//    if(digitalRead(SENSOR_C)){
//      Cseen1  = true;
//      TMRArd_InitTimer(TIMER_0, JIGGLE_INTERVAL); 
//    }
//  } else if (!TMRArd_IsTimerExpired(TIMER_0) && !Bseen1 && !Eseen1){
//    analogWrite(motorStateOut, spinCC);
//      if(digitalRead(SENSOR_B)){
//        Bseen1 = true;
//      }
//      if(digitalRead(SENSOR_E)){
//        Eseen1 = true; 
//      }
//    }
//  else if(!Bseen1 && !Eseen1){
//    analogWrite(motorStateOut, spinCl);
//    if(digitalRead(SENSOR_D)){
//      
//    }
//  }
  
}
*/
