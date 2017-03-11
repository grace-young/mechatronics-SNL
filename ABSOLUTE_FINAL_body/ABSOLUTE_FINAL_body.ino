#include <Timers.h>
#include <Wire.h>

#define MotorControlPin 2
#define MotorSpeedControlPin 3
#define COMMS_OUT_TO_BRAIN 9

#define TIME_INTERVAL      500
#define TIMER_0            0

#define WHEEL_ONE_L 7
#define WHEEL_ONE_R 8

#define WHEEL_TWO_L A1
#define WHEEL_TWO_R A0

#define WHEEL_THREE_L 13
#define WHEEL_THREE_R 12

#define WHEEL_FOUR_L A2
#define WHEEL_FOUR_R 4

// each enable pin must be able to analogWrite to
#define WHEEL_ONE_ENABLE     10
#define WHEEL_TWO_ENABLE     6
#define WHEEL_THREE_ENABLE   11
#define WHEEL_FOUR_ENABLE    5

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

//commands to Brain 
#define   GYRO_NEGATIVE             20
#define   GYRO_POSITIVE             40

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
static int motor_speed1_normal= 150;
static int motor_speed2_normal= 150;  
static int motor_speed3_normal= 150;  
static int motor_speed4_normal= 150;

static int motor_speed1_slow= 100; // this was 80
static int motor_speed2_slow= 100;  // then it was 75
static int motor_speed3_slow= 100;  
static int motor_speed4_slow= 100; // might have to do fancy rev thing


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

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Gyro Configuration
  // Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  
  SetupPins();
  jiggleClockwise = true;
  
  attachInterrupt(digitalPinToInterrupt(MotorControlPin), findFreqMotorControl, RISING);
  attachInterrupt(digitalPinToInterrupt(MotorSpeedControlPin), findFreqMotorSpeedControl, RISING);
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
  //goForwardCrossDir();
  decodeSignalsFromBrain();
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
//  Serial.println(gz,DEC);  
//  Serial.print ("\t");
//  Serial.print (prevZ);
//  Serial.print("   ");
  // 100 was nice 
 //Serial.print("orientation");
  //Serial.println(orientation);
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

void updateMotorSpeeds(){
  analogWrite(WHEEL_ONE_ENABLE, motor_speed1);
  analogWrite(WHEEL_TWO_ENABLE, motor_speed2);
  analogWrite(WHEEL_THREE_ENABLE, motor_speed3);
  analogWrite(WHEEL_FOUR_ENABLE, motor_speed4);
//  Serial.println("Wrote speed:");
//  Serial.println(motor_speed1);  
}

void decodeSignalsFromBrain(){
  int motorstate = map(pwm_value_motor_control, 0, 1920, 0, 11);
  int motorspeed = map(pwm_value_motor_speed, 0, 1920, 0, 11);


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
      motor_speed1 = motor_speed1_normal;
      motor_speed2 = motor_speed2_normal;
      motor_speed3 = motor_speed3_normal;
      motor_speed4 = motor_speed4_normal;
      break;
    case 1://40
      //Serial.println("1");
      motor_speed1 = motor_speed1_slow;
      motor_speed2 = motor_speed2_slow;
      motor_speed3 = motor_speed3_slow;
      motor_speed4 = motor_speed4_slow;
      break;
    case 2://60
      //Serial.println("2");
      motor_speed1 = motor_speed1_fast;
      motor_speed2 = motor_speed2_fast;
      motor_speed3 = motor_speed3_fast;
      motor_speed4 = motor_speed4_fast;
      break;
     case 3:
      //Serial.println("2");
      orientation = 0;
      break;
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
      goBackwardsCrossDirJiggle();
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

void SetupPins() {  
  pinMode(MotorControlPin, INPUT);
  pinMode(MotorSpeedControlPin, INPUT);
  
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

void rotateCounterClockwise() {
  turnWheelFourClockwise();
  turnWheelTwoClockwise();
  turnWheelThreeClockwise();
  turnWheelOneClockwise();
}

void rotateClockwise() {
  turnWheelFourCounterClockwise();
  turnWheelTwoCounterClockwise();
  turnWheelThreeCounterClockwise();
  turnWheelOneCounterClockwise();
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

void communicateGyroInfo(){
  if(orientation <= 0){
    analogWrite(COMMS_OUT_TO_BRAIN,GYRO_NEGATIVE);
    //Serial.println("negative");
  } else if(orientation > 0){
    analogWrite(COMMS_OUT_TO_BRAIN,GYRO_POSITIVE);
    //Serial.println("positive");
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
