#include <Timers.h>
#include <Pulse.h>

//tape sensors

#define PIN_SENSOR_A 7 
#define PIN_SENSOR_B_1 5
#define PIN_SENSOR_C 8
#define PIN_SENSOR_D_1 13
#define PIN_SENSOR_E 12
#define PIN_SENSOR_B_2 6
#define PIN_SENSOR_D_2 11
#define PIN_SENSOR_F  217 //TODO

#define PIN_GYRO_COMM 420

#define PIN_GAME_START           10

#define FLYWHEEL_ON A1
#define PULSE_STEPPER 7

#define STEPPER_PERIOD 200

#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 1000

#define TIME_BETWEEN_LINE_TAPE_READS  500
#define TIME_JIGGLE_BACKWARDS         1000
#define TIME_MOVE_LEFT                1000
#define TIME_GO_BACK                  1000
#define TIME_GO_RIGHT                 1000
#define TIME_BLOCK                    1000
#define TIME_RELOAD                   1000

static int numLinesCounted;
static unsigned long timer;
bool stopShooting=true;

typedef enum{
  WAIT, START, ORIENTATION_STRAIGHT, SLAM_LEFT, 
  LOOK_FOR_TAPE, SHOOT, 
  HEAD_BACK, SLAM_RIGHT, SLAM_PRESSURE, SLAM_DOWN,
  RELOAD
} States_t;

bool startGyroNegative=false;

States_t state; 
void setup() {
  Serial.begin(57600);
  state=WAIT;
  SetupPins();

  timer=millis();

  CoastStop();

}

void loop() {
  CheckGlobalStates();
}

void SetupPins(){
  pinMode(PIN_SENSOR_A, INPUT);
  pinMode(PIN_SENSOR_B_1, INPUT);
  pinMode(PIN_SENSOR_B_2, INPUT);
  pinMode(PIN_SENSOR_C, INPUT);
  pinMode(PIN_SENSOR_D_1, INPUT);
  pinMode(PIN_SENSOR_D_2, INPUT);
  pinMode(PIN_SENSOR_E, INPUT);
  pinMode(PIN_SENSOR_F, INPUT);

  pinMode(PIN_GAME_START, INPUT_PULLUP);

  pinMode(PULSE_STEPPER, OUTPUT);
  pinMode(FLYWHEEL_ON, OUTPUT);
}
void CheckGlobalStates(){
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
    case SLAM_LEFT:
      RespSlamLeft();
      break;
    case LOOK_FOR_TAPE:
      RespLookForTape();
      break;
    case SHOOT:
      RespShoot();
      break;
    case HEAD_BACK:
      RespHeadBack();
      break;
    case SLAM_RIGHT:
      RespSlamRight();
      break;
    case SLAM_PRESSURE:
      RespSlamPressure();
      break;
    case SLAM_DOWN:
      RespSlamDown();
      break;
    case RELOAD:
      RespReload();
      break;
    default:
      break;
  }
}

void RespWait(){
  //checks which way gyro is now facing
  if (ReadGyro() == 0){
    startGyroNegative=true;
  } else {
    startGyroNegative=false;
  }
  
  if (!digitalRead(PIN_GAME_START)){
    state=START;
    MotorSpeedNormal();
  }
}

void RespStart(){
  int gyroRead=Comms();
  if (startGyroNegative && (ReadGyro()==0)){
    SpinCC();
  } else if (!startGyroNegative && (ReadGyro()==1)){
    SpinCL();
  } else {
    state=ORIENTATION_STRAIGHT;
    MotorSpeedFast();
    JiggleBackwards();
    timer=millis();
  }
}

void RespOrientationStraight(){
  if (millis()-timer >=TIME_JIGGLE_BACKWARDS){
    state=SLAM_LEFT;
    MoveLeft();
    ExtendEye();
  }
}
void RespSlamLeft(){
  //or limit switch
  if (millis() -timer>=TIME_MOVE_LEFT){
    MotorSpeedSlow();
    MoveForwards();
    state=LOOK_FOR_TAPE;
    timer=millis();
  }
}
void RespLookForTape(){
  //check to make sure not looking for same line
  if (millis()-timer >= TIME_BETWEEN_LINE_TAPE_READS){
    if (ReadTapeSensor_F()){
      numLinesCounted++;
      Stop();
      state=SHOOT;
    }
  }
}
void RespShoot(){
  Shoot();
  if (stopShooting){
    //if it's only counted first/second line, keep going
    //change this to decrease number of lines counting before going backwards
    if (numLinesCounted<3){
      state=LOOK_FOR_TAPE;
    } else {
      state=HEAD_BACK;
      MotorSpeedFast();
      MoveBackwards();
      RetractEye();
      numLinesCounted=0;
      timer=millis();
    }
  }
}
void RespHeadBack(){
  //limit switch for the back!!!!
  //if (millis()-timer>=TIME_GO_BACK || limitSwitchBack()){
  if (millis()-timer >= TIME_GO_BACK){
    MoveRight();
    timer=millis();
    state=SLAM_RIGHT;
  }
}
void RespSlamRight(){
  //limit switch for right??
  if (millis()-timer >= TIME_GO_RIGHT){
    MoveForwards();
    timer=millis();
    state=SLAM_PRESSURE;
  }
}
void RespSlamPressure(){
  //limit sensor for hitting the wooden brackets between towers
  if (millis()-timer >= TIME_BLOCK){
    MoveBackwards();
    timer=millis();
    state=SLAM_DOWN;
  }
}
void RespSlamDown(){
  //limit swtich from tower pressure sensor to back short wall
  if (millis()-timer >= TIME_BLOCK){
    MoveLeft();
    timer=millis();
    MotorSpeedSlow();
    state=RELOAD;
  }
}
void RespReload(){
  //make sure fully in box!!!
  if (ReadTapeSensor_D_1() || ReadTapeSensor_D_2()){
    Stop();
    
    if (millis()-timer >=TIME_RELOAD){
      MoveLeft();
      timer=millis();
      MotorSpeedFast();
      state=SLAM_LEFT; //entire process repeats
    }
    
  }
}


void ExtendEye(){
  //tell servo for tape sensor to extend out
  
}
void RetractEye(){
  //tell servo for tape sensor to retract back
}
void Shoot(){
  //servo and flywheel code
  //some case to tell it to stop shooting
  stopShooting=true;
}


//drive base code
void MoveForwards(){

}
void MoveBackwards(){
  
}
void MoveLeft(){
  
}
void MoveRight(){
  
}
void Stop(){
  
}
void CoastStop(){
  
}
void SpinCL(){
  
}
void SpinCC(){
  
}
void JiggleBackwards(){
  
}
void MotorSpeedFast(){
  
}
void MotorSpeedNormal(){
  
}
void MotorSpeedSlow(){
  
}

//communication codes
int Comms(){
  return 0;
}

bool ReadGyro(){
  return digitalRead(PIN_GYRO_COMM);
}

//tape sensors code
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
bool ReadTapeSensor_F(void){
  return digitalRead(PIN_SENSOR_F);
}

