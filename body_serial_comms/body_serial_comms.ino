#include <Timers.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define MotorControlPin 2
#define MotorSpeedControlPin 3
#define COMMS_OUT_TO_BRAIN 9


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

volatile int pwm_value_motor_control = 0;
volatile int prev_time_motor_control = 0;
volatile int pwm_value_motor_speed = 0;
volatile int prev_time_motor_speed = 0;

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


//commands to Brain 
const char GYRO_NEGATIVE              = 'n';
const char GYRO_POSITIVE              = 'o';


/*
Not all pins on the Mega and Mega 2560
support change interrupts, so only the 
following can be used for RX: 10, 11, 12, 
13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63),
A10 (64), A11 (65), A12 (66), A13 (67),
A14 (68), A15 (69).
*/

// this is RX
#define PIN_COMMS_IN_HERE_RX    0 // number goes here!!!
// this is TX
//#define PIN_COMMS_OUT_HERE_TX   9 // number goes here!!!!

//SoftwareSerial bodySerial(PIN_COMMS_IN_HERE_RX, PIN_COMMS_OUT_HERE_TX);

void setup() {
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
    SetupPins();
  // this is the sending serial
<<<<<<< HEAD
  Serial.begin(4800); // 115200 USE FASTEST SETTING FOR COMMS
//  while(!Serial.available()){
//    ;
//  }
=======
  Serial.begin(57600); // 115200 USE FASTEST SETTING FOR COMMS
  /*while(!Serial){
      ; // wait for serial port to connect. Needed for native USB port only
      // from example
  }
// do Serial's have to be at different rates??
  bodySerial.begin(4800); // why is this low?????
  // this is the receiving serial
  */
=======
  Serial.begin(57600);
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
>>>>>>> 5dd5c011dc986fc9bd2bc07ea1484422dffc2f49
  Wire.begin();

  //Gyro Configuration
  // Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
=======
  SetupPins();
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
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
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
//  decodeSignalsFromBrain();
  //goForwardCrossDir();
//  communicateGyroInfo();
=======
  decodeSignalsFromBrain();
  //goForwardCrossDir();
  communicateGyroInfo();
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
  
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

<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
=======
//  Serial.print("ax ");
//  Serial.print(ax, DEC);
//
//  Serial.print("ay ");
//  Serial.print(ay,DEC);
//
//  Serial.print("az ");
//  Serial.print(az,DEC);
//  Serial.print(" ");
//  Serial.print ("x ");
//  Serial.print (gx,DEC); 
//  Serial.print ("\t");
//  Serial.print ("y ");
//  Serial.print (gy,DEC);
//  Serial.print ("\t");
//  Serial.print("z ");
//  Serial.print(gz,DEC);  
//  Serial.print ("\t");
//  Serial.print (prevZ);
//  Serial.print("   ");
  // 100 was nice 
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
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
    
         digitalWrite(10,HIGH);
         //delay(1000);
        //Serial.println(inChar);
    }
}


void updateMotorSpeeds(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);
}

void decodeSignalsFromBrain(char comm_state){
  /*char comm_state = 'z';
  if(bodySerial.available()){
     comm_state = bodySerial.read(); 
     //Serial.println(comm_state);
  }
  */

<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
  switch(comm_state){
    case COMM_MOTOR_NORMAL:
=======
  // 20 --> 0 normal speed
  // 40 --> 1 slow speed
  // 60 --> 2 fast speed 
  // 80 --> 3 clear gyro
  // UNCLEAR IF THIS WORKS OR NOT
//  Serial.println(pwm_value_motor_control);
  Serial.println(motorstate);
  switch(motorspeed){
    case 0://20
      //Serial.println("0");
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
      motor_speed1 = motor_speed1_normal;
      motor_speed2 = motor_speed2_normal;
      motor_speed3 = motor_speed3_normal;
      motor_speed4 = motor_speed4_normal;
      break;
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
    case COMM_MOTOR_SLOW:
=======
    case 1://40
      //Serial.println("1");
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
      motor_speed1 = motor_speed1_slow;
      motor_speed2 = motor_speed2_slow;
      motor_speed3 = motor_speed3_slow;
      motor_speed4 = motor_speed4_slow;
      break;
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
    case COMM_MOTOR_FAST:
=======
    case 2://60
      //Serial.println("2");
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
      motor_speed1 = motor_speed1_fast;
      motor_speed2 = motor_speed2_fast;
      motor_speed3 = motor_speed3_fast;
      motor_speed4 = motor_speed4_fast;
      break;
     case COMM_ZERO_GYRO:
      orientation = 0;
      break;
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
    case COMM_MOTOR_COAST_STOP:
      // pin to test wait
      digitalWrite(10, HIGH);
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
=======
    default:
      //Serial.println("default");
      motor_speed1 = motor_speed1_slow;
      motor_speed2 = motor_speed2_slow;
      motor_speed3 = motor_speed3_slow;
      motor_speed4 = motor_speed4_slow;
      break;
  }
  
  switch(motorstate){
    case 0://turn everything off 20
      coastStopAll();
      break;
    case 1://40
      goForwardCrossDir();
      //goForwardOmniDir();
      break;
    case 2://60
      goBackwardsCrossDir();
      //goBackwardsOmniDir();
      break;
    case 3://80
      goLeftCrossDir();
      //goLeftOmniDir();
      break;
    case 4://100
      goRightCrossDir();
      //goRightOmniDir();
      break;
    case 5://120
      rotateCounterClockwise();
      break;
    case 6://140
      rotateClockwise();
      break;
    case 7://160
      stopAllWheels();
      break;
    case 8: //180
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
      goBackwardsCrossDirJiggle();
      break;
    default:
      break;
  }
}

<<<<<<< HEAD
void SetupPins() {   
  pinMode(10, OUTPUT);
   
=======
void SetupPins() {  
  pinMode(MotorControlPin, INPUT);
  pinMode(MotorSpeedControlPin, INPUT);
  
>>>>>>> 5dd5c011dc986fc9bd2bc07ea1484422dffc2f49
  pinMode(PIN_COMMS_IN_HERE_RX, INPUT);
  //pinMode(PIN_COMMS_OUT_HERE_TX, OUTPUT);
  
  
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

/*
void communicateGyroInfo(){
  if(orientation <= 0){
<<<<<<< HEAD:body_serial_comms/body_serial_comms.ino
    Serial.write(GYRO_NEGATIVE);
  } else if(orientation > 0){
    Serial.write(GYRO_POSITIVE);
=======
    analogWrite(COMMS_OUT_TO_BRAIN,GYRO_NEGATIVE);
  } else if(orientation > 0){
    analogWrite(COMMS_OUT_TO_BRAIN,GYRO_POSITIVE);
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f:body_with_start_sequence/body_with_start_sequence.ino
  }
}
*/

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
