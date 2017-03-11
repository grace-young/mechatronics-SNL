#include <Timers.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define MotorControlPin 2
#define MotorSpeedControlPin 3



#define TIME_INTERVAL      500
#define TIMER_0            0

#define WHEEL_FOUR_L 4
#define WHEEL_FOUR_R A2

#define WHEEL_TWO_L A1
#define WHEEL_TWO_R A0

#define WHEEL_THREE_L 12
#define WHEEL_THREE_R 13

#define WHEEL_ONE_L 8
#define WHEEL_ONE_R 7

// each enable pin must be able to analogWrite to
#define WHEEL_ONE_ENABLE     3
#define WHEEL_TWO_ENABLE     6
#define WHEEL_THREE_ENABLE   11
#define WHEEL_FOUR_ENABLE    5

#define LED1  2
#define LED2  3
#define LED3  9
#define LED4  A3

#define LIMIT_ON_ANALOG      255

//Gyro Stuff
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
//Gyro stuff ends

static int motor_speed1= 155;
static int motor_speed2= 203;  
static int motor_speed3= 165;  
static int motor_speed4= 173;

static int motor_speed1_fast= 250;
static int motor_speed2_fast= 250;  
static int motor_speed3_fast= 250;  
static int motor_speed4_fast= 250;

// these used to be the normal values, but 
// now they are the fast ones.
static int motor_speed1_normal= 85;
static int motor_speed2_normal= 85;  
static int motor_speed3_normal= 85;  
static int motor_speed4_normal= 85;

static int motor_speed1_slow= 90; // this was 80
static int motor_speed2_slow= 90;  // then it was 75
static int motor_speed3_slow= 90;  
static int motor_speed4_slow= 90; // might have to do fancy rev thing

unsigned long time_last_printed;

unsigned long jiggle_time;
unsigned long test_time;
bool jiggleClockwise;

//Gyro variables
long orientation = 0;
int prevZ;
bool flag;
long int cpt=0; //count
bool statedPositive;
//Gyro variables end

// NEW CONSTANTS
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


// this is RX
#define PIN_COMMS_IN_HERE_RX    0 // number goes here!!!

#define PIN_GYRO                9

void setup() {
    SetupPins();
  // this is the sending serial
  Serial.begin(57600); // 115200 USE FASTEST SETTING FOR COMMS
  
  Wire.begin();

  //Gyro Configuration
  // Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  
  jiggleClockwise = true;
  
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);  

  // init fake timers
  time_last_printed = millis();
  jiggle_time = millis();
  test_time = millis();
}

// TODO: MAKE SURE WE ARE ACTUALLY CHANGING SPEED WHEN CHANGING DIRECTION TOO

void loop() {
  // now signals from brain happens asynchronusly
  communicateGyroInfo();
  
////////////GYRO

//  Serial.print (cpt++,DEC); //Used to debug gyro
//  Serial.print ("\t");
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Create 16 bits values from 8 bits data
  
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

   // Accelerometer
//  int16_t ax=-(Buf[0]<<8 | Buf[1]);
//  int16_t ay=-(Buf[2]<<8 | Buf[3]);
//  int16_t az=Buf[4]<<8 | Buf[5];

//  Serial.print("orientation");
//  Serial.println(orientation);
  flag = false;
  if ( (abs(gx) > 600 || abs(gy) > 600)) {
    flag = true;
  }
  if((gz<0 && prevZ >0) && (gz>0 && prevZ <0)&&abs(gz) > 15){
    orientation -= prevZ/10;
  }
  else if ( (abs(gx) > 600 || abs(gy) > 600) && abs(gz) > 100) {
    orientation += gz/10;
  }
  else if (abs(gz) > 15 && !flag) {
    orientation += gz/10;
  }
  prevZ = gz;

  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  // Create 16 bits values from 8 bits data
  
  // End of line
//  Serial.println("");

////////////GYRO END
}

void serialEvent(){
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    decodeSignalsFromBrain(inChar);   
    }
}




void updateMotorSpeeds(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);
}

void decodeSignalsFromBrain(char comm_state){
  switch(comm_state){
    case COMM_MOTOR_NORMAL:
      motor_speed1 = motor_speed1_normal;
      motor_speed2 = motor_speed2_normal;
      motor_speed3 = motor_speed3_normal;
      motor_speed4 = motor_speed4_normal;
      break;
    case COMM_MOTOR_SLOW:
      motor_speed1 = motor_speed1_slow;
      motor_speed2 = motor_speed2_slow;
      motor_speed3 = motor_speed3_slow;
      motor_speed4 = motor_speed4_slow;
      break;
    case COMM_MOTOR_FAST:
      motor_speed1 = motor_speed1_fast;
      motor_speed2 = motor_speed2_fast;
      motor_speed3 = motor_speed3_fast;
      motor_speed4 = motor_speed4_fast;
      break;
     case COMM_ZERO_GYRO:
      orientation = 0;
      break;
    case COMM_MOTOR_COAST_STOP:
      coastStopAll();
      break;
    case COMM_MOTOR_FORWARD:
      goForwardCrossDir();
      break;
    case COMM_MOTOR_BACKWARD:
      goBackwardsCrossDir();
      break;
    case COMM_MOTOR_LEFT:
      goLeftCrossDir();
      break;
    case COMM_MOTOR_RIGHT:
      goRightCrossDir();
      break;
    case COMM_MOTOR_SPIN_CC:
      rotateCounterClockwise();
      break;
    case COMM_MOTOR_SPIN_CL:
      rotateClockwise();
      break;
    case COMM_MOTOR_STOP:
      stopAllWheels();
      break;
    case COMM_MOTOR_BACKWARD_JIGGLE: 
      goBackwardsCrossDirJiggle();
      break;
    default:
      break;
  }
}

void SetupPins() {    
  pinMode(PIN_COMMS_IN_HERE_RX, INPUT);
  
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

  pinMode(PIN_GYRO, OUTPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
}

void coastStopAll(){
  digitalWrite(WHEEL_ONE_ENABLE, LOW);
  digitalWrite(WHEEL_TWO_ENABLE, LOW);
  digitalWrite(WHEEL_THREE_ENABLE, LOW);
  digitalWrite(WHEEL_FOUR_ENABLE, LOW);
}

void coastStopWheelOne(){
    digitalWrite(WHEEL_ONE_ENABLE, LOW);
}

void coastStopWheelTwo(){
    digitalWrite(WHEEL_TWO_ENABLE, LOW);
}


void coastStopWheelThree(){
    digitalWrite(WHEEL_THREE_ENABLE, LOW);
}


void coastStopWheelFour(){
    digitalWrite(WHEEL_FOUR_ENABLE, LOW);
}

/* =====================================
 *  Drive forward and backward using cross
 * ===================================== */

void goForwardCrossDir(){
    turnWheelTwoCounterClockwise(); // this works
    turnWheelFourClockwise();
    coastStopWheelOne();
    coastStopWheelThree();
}

void goBackwardsCrossDir(){
    turnWheelTwoClockwise();
    turnWheelFourCounterClockwise();
    coastStopWheelOne();
    coastStopWheelThree();
}

void goBackwardsCrossDirJiggle(){
    turnWheelTwoClockwise();
    turnWheelFourCounterClockwise();
    if(jiggleClockwise){
      turnWheelOneClockwise();
      if(millis() - jiggle_time > 100){
        jiggleClockwise = !jiggleClockwise;
        jiggle_time = millis();
      }
    }else{
      turnWheelOneCounterClockwise();
      if(millis() - jiggle_time > 100){
        jiggleClockwise = !jiggleClockwise;
        jiggle_time = millis();
      }
    } 
    coastStopWheelThree();
}

/* =====================================
 *  Drive right and left using cross
 * ===================================== */
 
void goLeftCrossDir(){
  // wheel 2 --> clockwise
  turnWheelOneClockwise();
  //wheel 4 --> counterclockwise
  turnWheelThreeCounterClockwise();
  coastStopWheelTwo();
  coastStopWheelFour();
}

void goRightCrossDir(){
  // wheel 2 --> clockwise
  turnWheelOneCounterClockwise();
  //wheel 4 --> counterclockwise
  turnWheelThreeClockwise();
  coastStopWheelTwo();
  coastStopWheelFour();
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
  digitalWrite(WHEEL_THREE_R, HIGH);
  digitalWrite(WHEEL_THREE_L, LOW);
}

void turnWheelThreeCounterClockwise(){
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  digitalWrite(WHEEL_THREE_R, LOW);
  digitalWrite(WHEEL_THREE_L, HIGH);
}

//Wheel One

void turnWheelOneClockwise(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  digitalWrite(WHEEL_ONE_R, LOW);
  digitalWrite(WHEEL_ONE_L, HIGH);
}

void turnWheelOneCounterClockwise(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  digitalWrite(WHEEL_ONE_R, HIGH);
  digitalWrite(WHEEL_ONE_L, LOW);
}

/* =====================================
 *  STOPPING WHEELS
 * ===================================== */

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


void communicateGyroInfo(){
  if(orientation <= 0){
    digitalWrite(PIN_GYRO, LOW);
  } else if(orientation > 0){
    digitalWrite(PIN_GYRO, HIGH);
  }
}


//Gyro Functions
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
