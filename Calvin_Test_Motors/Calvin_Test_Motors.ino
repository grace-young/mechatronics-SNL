#include <Timers.h>

#define MotorControlPin 2
#define MotorSpeedControlPin 3

#define SENSOR_1 A3
#define SENSOR_2 A5
#define SENSOR_3 A4
#define SENSOR_4 A2
#define SENSOR_5 A1

#define TIME_INTERVAL      500
#define TIMER_0            0

#define WHEEL_FOUR_L 4
#define WHEEL_FOUR_R A1

#define WHEEL_TWO_L 6
#define WHEEL_TWO_R A0

#define WHEEL_THREE_L 12
#define WHEEL_THREE_R 13

#define WHEEL_ONE_L 8
#define WHEEL_ONE_R 7

// each enable pin must be able to analogWrite to
#define WHEEL_ONE_ENABLE     10
#define WHEEL_TWO_ENABLE     9
#define WHEEL_THREE_ENABLE   11
#define WHEEL_FOUR_ENABLE    5

#define LIMIT_ON_ANALOG      255

static int motor_speed1= 155;
static int motor_speed2= 203;  
static int motor_speed3= 165;  
static int motor_speed4= 173;

static int motor_speed1_fast= 255;
static int motor_speed2_fast= 255;  
static int motor_speed3_fast= 255;  
static int motor_speed4_fast= 255;

static int motor_speed1_normal= 155;
static int motor_speed2_normal= 203;  
static int motor_speed3_normal= 165;  
static int motor_speed4_normal= 173;

static int motor_speed1_slow= 80;
static int motor_speed2_slow= 80;  
static int motor_speed3_slow= 80;  
static int motor_speed4_slow= 80;

static bool SawTape;    

volatile int pwm_value_motor_control = 0;
volatile int prev_time_motor_control = 0;
volatile int pwm_value_motor_speed = 0;
volatile int prev_time_motor_speed = 0;

void setup() {
  SawTape = false;
  Serial.begin(230400);
  TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  SetupPins();
  attachInterrupt(digitalPinToInterrupt(MotorControlPin), findFreqMotorControl, RISING);
  attachInterrupt(digitalPinToInterrupt(MotorSpeedControlPin), findFreqMotorSpeedControl, RISING);
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);  
}


void loop() {
  decodeSignalsFromBrain();
  //goRightOmniDir();
 // if(!SawTape){
   // goForwardOmniDir();
  //}
  //else{
  //  stopAllWheels();
  //}
  if(!(digitalRead(SENSOR_1) && digitalRead(SENSOR_2) && digitalRead(SENSOR_3) && digitalRead(SENSOR_4) && digitalRead(SENSOR_5))){
    SawTape = true;
  }
  //printTapeDate();
}

void decodeSignalsFromBrain(){
  int motorstate = map(pwm_value_motor_control, 0, 1920, 0, 11);
  int motorspeed = map(pwm_value_motor_speed, 0, 1920, 0, 2);
//  Serial.println(pwm_value_motor_control);
//  Serial.println(state);
  switch(motorspeed){
    case 0:
      motor_speed1 = motor_speed1_normal;
      motor_speed2 = motor_speed2_normal;
      motor_speed3 = motor_speed3_normal;
      motor_speed4 = motor_speed4_normal;
      break;
    case 1:
      motor_speed1 = motor_speed1_slow;
      motor_speed2 = motor_speed2_slow;
      motor_speed3 = motor_speed3_slow;
      motor_speed4 = motor_speed4_slow;
      break;
    case 2:
      motor_speed1 = motor_speed1_fast;
      motor_speed2 = motor_speed2_fast;
      motor_speed3 = motor_speed3_fast;
      motor_speed4 = motor_speed4_fast;
      break;
    default:
      motor_speed1 = motor_speed1_slow;
      motor_speed2 = motor_speed2_slow;
      motor_speed3 = motor_speed3_slow;
      motor_speed4 = motor_speed4_slow;
      break;
  }
  
  switch(motorstate){
    case 0://turn everything off
      coastStopAll();
      break;
    case 1:
      goForwardOmniDir();
      break;
    case 2:
      goBackwardsOmniDir();
      break;
    case 3:
      goLeftOmniDir();
      break;
    case 4:
      goRightOmniDir();
      break;
    case 5:
      rotateCounterClockwise();
      break;
    case 6:
      rotateClockwise();
      break;
    case 7:
      stopAllWheels();
      break;
    default:
      coastStopAll();
      break;
  }
}

void findFreqMotorControl(){
  attachInterrupt(digitalPinToInterrupt(MotorControlPin), freqCountMotorControl, FALLING);
  prev_time_motor_control = micros();
}
void freqCountMotorControl(){
  attachInterrupt(digitalPinToInterrupt(MotorControlPin), findFreqMotorControl, RISING);
  pwm_value_motor_control = micros() - prev_time_motor_control;
//  Serial.print("MOTOR CONTROL ");
//  Serial.println(pwm_value_motor_control);
}

void findFreqMotorSpeedControl(){
  attachInterrupt(digitalPinToInterrupt(MotorSpeedControlPin), freqCountMotorSpeedControl, FALLING);
  prev_time_motor_speed = micros();
}
void freqCountMotorSpeedControl(){
  attachInterrupt(digitalPinToInterrupt(MotorSpeedControlPin), findFreqMotorSpeedControl, RISING);
  pwm_value_motor_speed = micros() - prev_time_motor_speed;
//   Serial.print("MOTOR SPEED ");
//  Serial.println(pwm_value_motor_speed);
}

void printTapeDate() {
  if (TMRArd_IsTimerExpired(TIMER_0)){
      Serial.print("Sensor One: ");
      Serial.println(digitalRead(SENSOR_1));
      
      Serial.print("Sensor Two: ");
      Serial.println(digitalRead(SENSOR_2));
      
      Serial.print("Sensor Three: ");
      Serial.println(digitalRead(SENSOR_3));
      
      Serial.print("Sensor Four: ");
      Serial.println(digitalRead(SENSOR_4));

      Serial.print("Sensor Five: ");
      Serial.println(digitalRead(SENSOR_5));
      
      TMRArd_InitTimer(TIMER_0, TIME_INTERVAL); 
  }
}

void SetupPins() {  
  pinMode(MotorControlPin, INPUT);
  pinMode(MotorSpeedControlPin, INPUT);
  
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  
  pinMode(WHEEL_ONE_ENABLE, OUTPUT);
  pinMode(WHEEL_TWO_ENABLE, OUTPUT);
  pinMode(WHEEL_THREE_ENABLE, OUTPUT);
  pinMode(WHEEL_FOUR_ENABLE, OUTPUT);
  
  pinMode(WHEEL_FOUR_L, OUTPUT);
  pinMode(WHEEL_FOUR_R, OUTPUT);

  pinMode(WHEEL_THREE_L, OUTPUT);
  pinMode(WHEEL_THREE_R, OUTPUT);

  pinMode(WHEEL_TWO_L, OUTPUT);
  pinMode(WHEEL_TWO_R, OUTPUT);

  pinMode(WHEEL_ONE_L, OUTPUT);
  pinMode(WHEEL_ONE_R, OUTPUT);
}

void coastStopAll(){
  digitalWrite(WHEEL_ONE_ENABLE, LOW);
  digitalWrite(WHEEL_TWO_ENABLE, LOW);
  digitalWrite(WHEEL_THREE_ENABLE, LOW);
  digitalWrite(WHEEL_FOUR_ENABLE, LOW);
}

void goForwardOmniDir(){
  turnWheelThreeCounterClockwise();
  turnWheelTwoClockwise();
  turnWheelFourCounterClockwise();
  turnWheelOneClockwise();
}

void goBackwardsOmniDir(){
  turnWheelThreeClockwise();
  turnWheelTwoCounterClockwise();
  turnWheelFourClockwise();
  turnWheelOneCounterClockwise();
}

void goRightOmniDir(){
  turnWheelThreeCounterClockwise();
  turnWheelTwoCounterClockwise();
  turnWheelFourClockwise();
  turnWheelOneClockwise();
}

void goLeftOmniDir(){
  turnWheelThreeClockwise();
  turnWheelTwoClockwise();
  turnWheelFourCounterClockwise();
  turnWheelOneCounterClockwise();
}

void rotateClockwise() {
  turnWheelFourCounterClockwise();
  turnWheelTwoCounterClockwise();
  turnWheelThreeCounterClockwise();
  turnWheelOneCounterClockwise();
}

void rotateCounterClockwise() {
  turnWheelFourClockwise();
  turnWheelTwoClockwise();
  turnWheelThreeClockwise();
  turnWheelOneClockwise();
}

//Wheel Four

void turnWheelFourClockwise(){
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);
  digitalWrite(WHEEL_FOUR_R, HIGH);
  digitalWrite(WHEEL_FOUR_L, LOW);
}

void turnWheelFourCounterClockwise(){
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);
  digitalWrite(WHEEL_FOUR_R, LOW);
  digitalWrite(WHEEL_FOUR_L, HIGH);
}

//Wheel Two

void turnWheelTwoClockwise(){
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  digitalWrite(WHEEL_TWO_R, LOW);
  digitalWrite(WHEEL_TWO_L, HIGH);
}

void turnWheelTwoCounterClockwise(){
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  digitalWrite(WHEEL_TWO_R, HIGH);
  digitalWrite(WHEEL_TWO_L, LOW);
}

//Wheel Three

void turnWheelThreeClockwise(){
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  digitalWrite(WHEEL_THREE_R, LOW);
  digitalWrite(WHEEL_THREE_L, HIGH);
}

void turnWheelThreeCounterClockwise(){
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  digitalWrite(WHEEL_THREE_R, HIGH);
  digitalWrite(WHEEL_THREE_L, LOW);
}

//Wheel One

void turnWheelOneClockwise(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  digitalWrite(WHEEL_ONE_R, HIGH);
  digitalWrite(WHEEL_ONE_L, LOW);
}

void turnWheelOneCounterClockwise(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  digitalWrite(WHEEL_ONE_R, LOW);
  digitalWrite(WHEEL_ONE_L, HIGH);
}

void stopAllWheels() {
  stopWheelOne();
  stopWheelTwo();
  stopWheelThree();
  stopWheelFour();
}

void stopWheelOne() {
  digitalWrite(WHEEL_ONE_R, HIGH);
  digitalWrite(WHEEL_ONE_L, HIGH);
}

void stopWheelTwo() {
  digitalWrite(WHEEL_TWO_R, HIGH);
  digitalWrite(WHEEL_TWO_L, HIGH);
}

void stopWheelThree() {
  digitalWrite(WHEEL_THREE_R, HIGH);
  digitalWrite(WHEEL_THREE_L, HIGH);
}

void stopWheelFour() {
  digitalWrite(WHEEL_FOUR_R, HIGH);
  digitalWrite(WHEEL_FOUR_L, HIGH);
}

void serialEvent(){
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    Serial.println(inChar);
      if (inChar == 'L' || inChar == 'l') {
          // increase speed
          if(motor_speed1 + 10 <= LIMIT_ON_ANALOG){
             motor_speed1 = motor_speed1 + 10;
          }
      }
      else if (inChar == 'k' || inChar == 'K') {
          // decrease speed
          if(motor_speed1 - 10 >= 0){
            motor_speed1 = motor_speed1 - 10;
          }
    }
  }
}
