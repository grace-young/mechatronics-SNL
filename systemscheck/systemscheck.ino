#include <Timers.h>

#define PIN_SENSOR_D_2 13
#define PIN_SENSOR_B_2 A4//A0 // NEW
#define PIN_SENSOR_D_1 A3 //Not Working A5
#define PIN_SENSOR_EYE 12//A0//A4

#define PIN_COMMS_IN_GYRO        2
#define PIN_OUTPUT_MOTOR_CONTROL 10 //changed from 3
#define PIN_OUTPUT_SPEED_CONTROL 9

#define COMM_MOTOR_COAST_STOP 20
#define COMM_MOTOR_FORWARD 40
#define COMM_MOTOR_BACKWARD 60
#define COMM_MOTOR_LEFT 80
#define COMM_MOTOR_RIGHT 100
#define COMM_MOTOR_SPIN_CC 120
#define COMM_MOTOR_SPIN_CL 140
#define COMM_MOTOR_STOP 160
#define COMM_MOTOR_BACKWARD_JIGGLE 180

#define COMM_MOTOR_NORMAL  20
#define COMM_MOTOR_SLOW    40
#define COMM_MOTOR_FAST    60
#define COMM_ZERO_GYRO     80

#define PIN_SHOOT 5
#define PIN_DONE  6
#define PIN_SERVO 7

#define TIME  500
#define DELAY_TEST_TIME       2000

bool first;
static bool startGyroNegative = false;

volatile int prev_time_comms = 0;
volatile int pwm_value_comms = 0;

static unsigned long fake_timer;
void setup() {
  SetupPins();
  Serial.begin(57600);
  first=true;
}

String toString(bool boo){
  if (boo){
    return "true";
  } else {
    return "false";
  }
}

void loop() {
  

  if (first){
    // test that it can drive forward
    makeMotorsMoveForward();
    Serial.println("forward");
    delay(DELAY_TEST_TIME);
  
    makeMotorsMoveBackward();
    Serial.println("backward");
    delay(DELAY_TEST_TIME);
  
    makeMotorsMoveRight();
    Serial.println("right");
    delay(DELAY_TEST_TIME);
  
    makeMotorsMoveLeft();
    Serial.println("left");
    delay(DELAY_TEST_TIME);
  
    makeMotorsSpinCL();
    Serial.println("clockwise");
    delay(DELAY_TEST_TIME);
  
    makeMotorsSpinCC();
    Serial.println("counterclockwise");
    delay(DELAY_TEST_TIME);
  
    makeMotorsStop();
    Serial.println("stop");
    delay(DELAY_TEST_TIME);

    digitalWrite(PIN_SHOOT, HIGH);
    Serial.println("shoot");
    delay(DELAY_TEST_TIME);
    digitalWrite(PIN_SHOOT, LOW);

    
    digitalWrite(PIN_SERVO, HIGH);
    Serial.println("servo");
    delay(DELAY_TEST_TIME);
    digitalWrite(PIN_SERVO, LOW);
    
    first=false;
  }

  String toPrint="";
  toPrint=" B2: "+toString(ReadTapeSensor_B_2())+" D1: "+toString(ReadTapeSensor_D_1())+" D2: "+toString(ReadTapeSensor_D_2())
  +" Eye: "+toString(ReadTapeSensor_EYE());
  Serial.println(toPrint);
  

  // shooting the stepper & flywheel goes here

  // extend the eye here
  
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
  return commsState;
}
void SetupPins(){
  pinMode(PIN_SENSOR_B_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_D_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_D_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_EYE, INPUT_PULLUP);
  
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  pinMode(PIN_OUTPUT_SPEED_CONTROL, OUTPUT);
  //pinMode(PIN_INPUT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_COMMS_IN_GYRO, INPUT_PULLUP);

  pinMode(PIN_SHOOT, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_DONE, INPUT);
}

bool ReadTapeSensor_B_2(void){
    // 1 if read tape, 0 if not 
    return digitalRead(PIN_SENSOR_B_2);
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
  return digitalRead(PIN_SENSOR_EYE);
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



