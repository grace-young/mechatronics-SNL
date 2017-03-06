#define MotorControlPin 2
#define MotorSpeedControlPin  3

#define WHEEL_FOUR_L 4
#define WHEEL_FOUR_R A2

#define WHEEL_TWO_L A1
#define WHEEL_TWO_R A0

#define WHEEL_THREE_L 12
#define WHEEL_THREE_R 13

#define WHEEL_ONE_L 8
#define WHEEL_ONE_R 7

#define KP_L  0.125
#define KP_R  0.125

// each enable pin must be able to analogWrite to
#define WHEEL_ONE_ENABLE     10
#define WHEEL_TWO_ENABLE     6
#define WHEEL_THREE_ENABLE   11
#define WHEEL_FOUR_ENABLE    5

long gyroCurr=0;
long gyroTarget=0;

#define TIME_INTERVAL 

void setup() {
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

void loop() {
  int target_gyro=0;
  int curr_gyro=getGyroPosition();
  int speed=120; 
  turnLeftSide(speed-(curr_gyro-target_gyro*KP_L));
  turnRightSide(speed+(curr_gyro-target_gyro*KP_R));
}
void turnLeftSide(int speed){
  analogWrite(WHEEL_ONE_ENABLE, abs(speed));
  if (speed>=0){
    digitalWrite(WHEEL_ONE_R, LOW);
    digitalWrite(WHEEL_ONE_L, HIGH);
  } else {
    digitalWrite(WHEEL_ONE_R, HIGH);
    digitalWrite(WHEEL_ONE_L, LOW);
  }
}

void turnRightSide (int speed){
  analogWrite(WHEEL_THREE_ENABLE, abs(speed));
  if (speed>=0){
    digitalWrite(WHEEL_THREE_R, LOW);
    digitalWrite(WHEEL_THREE_L, HIGH);
  } else {
    digitalWrite(WHEEL_THREE_R, HIGH);
    digitalWrite(WHEEL_THREE_L, LOW);
  }
}

int getGyroPosition(){
  
}

