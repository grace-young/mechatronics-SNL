#define WHEEL_ONE_L 8
#define WHEEL_ONE_R 7

#define WHEEL_TWO_L A1
#define WHEEL_TWO_R A0

#define WHEEL_THREE_L 12
#define WHEEL_THREE_R 13

#define WHEEL_FOUR_L 4
#define WHEEL_FOUR_R A2

// each enable pin must be able to analogWrite to
#define WHEEL_ONE_ENABLE     10
#define WHEEL_TWO_ENABLE     6
#define WHEEL_THREE_ENABLE   11
#define WHEEL_FOUR_ENABLE    5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SetupPins();
}

void loop() {
  // put your main code here, to run repeatedly:
}


void serialEvent(){
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // NOW, when we get a Serial event, want to start over one 
    
    if (inChar == 'f') {
      goForwardCrossDir();
    }else if (inChar == 'b') {
      goBackwardsCrossDir();
    }else if (inChar == 'r') {
      goRightCrossDir();
    }else if (inChar == 'l') {
      goLeftCrossDir();
    } else if (inChar == 'c'){
      rotateClockwise();
    } else if (inChar == 'k'){
      rotateCounterClockwise();
    }
  }
}

/* =====================================
 *  Drive forward and backward using cross
 * ===================================== */

void goForwardCrossDir(){
    turnWheelOneCounterClockwise();
    turnWheelThreeClockwise();
    coastStopWheelTwo();
    coastStopWheelFour();
}

void goBackwardsCrossDir(){
    turnWheelOneClockwise();
    turnWheelThreeCounterClockwise();
    coastStopWheelTwo();
    coastStopWheelFour();
}

/* =====================================
 *  Drive right and left using cross
 * ===================================== */
 
void goRightCrossDir(){
  // wheel 2 --> clockwise
  turnWheelTwoClockwise();
  //wheel 4 --> counterclockwise
  turnWheelFourCounterClockwise();
  coastStopWheelOne();
  coastStopWheelThree();
}

void goLeftCrossDir(){
  // wheel 2 --> clockwise
  turnWheelTwoCounterClockwise();
  //wheel 4 --> counterclockwise
  turnWheelFourClockwise();
  coastStopWheelOne();
  coastStopWheelThree();
}

/* =====================================
 *  Turn All Wheels
 * ===================================== */

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

/* =====================================
 *  Turn Wheel One
 * ===================================== */

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

/* =====================================
 *  Turn Wheel Two
 * ===================================== */

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

/* =====================================
 *  Turn Wheel Three
 * ===================================== */

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

/* =====================================
 *  Turn Wheel Four
 * ===================================== */

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

/* =====================================
 *   COAST STOPPPING
 * ===================================== */

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

void SetupPins() {    
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

