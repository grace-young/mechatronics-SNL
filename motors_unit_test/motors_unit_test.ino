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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SetupPins();
}

void loop() {
  // put your main code here, to run repeatedly:
<<<<<<< HEAD
    goBackwardsCrossDir();
=======
    //goRightCrossDir();
    goForwardCrossDir();
    /*turnWheelTwoClockwise();
    turnWheelFourCounterClockwise();
    coastStopWheelOne();
    coastStopWheelThree();
    */
>>>>>>> e53b4f1d47f9e61096ec208a013bec41cdc8d15f
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

/* =====================================
 *  Drive right and left using cross
 * ===================================== */
 
void goRightCrossDir(){
  // wheel 2 --> clockwise
  turnWheelOneClockwise();
  //wheel 4 --> counterclockwise
  turnWheelThreeCounterClockwise();
  coastStopWheelTwo();
  coastStopWheelFour();
}

void goLeftCrossDir(){
  // wheel 2 --> clockwise
  turnWheelOneCounterClockwise();
  //wheel 4 --> counterclockwise
  turnWheelThreeClockwise();
  coastStopWheelTwo();
  coastStopWheelFour();
}

/* =====================================
 *  Turn All Wheels
 * ===================================== */

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

/* =====================================
 *  Turn Wheel One
 * ===================================== */

void turnWheelOneClockwise(){
  analogWrite(WHEEL_ONE_ENABLE, 200);
  digitalWrite(WHEEL_ONE_R, LOW);
  digitalWrite(WHEEL_ONE_L, HIGH);
}

void turnWheelOneCounterClockwise(){
  analogWrite(WHEEL_ONE_ENABLE, 200);
  digitalWrite(WHEEL_ONE_R, HIGH);
  digitalWrite(WHEEL_ONE_L, LOW);
}

/* =====================================
 *  Turn Wheel Two
 * ===================================== */

void turnWheelTwoClockwise(){
  analogWrite(WHEEL_TWO_ENABLE, 200);
  digitalWrite(WHEEL_TWO_R, LOW);
  digitalWrite(WHEEL_TWO_L, HIGH);
}

void turnWheelTwoCounterClockwise(){
  analogWrite(WHEEL_TWO_ENABLE, 200);
  digitalWrite(WHEEL_TWO_R, HIGH);
  digitalWrite(WHEEL_TWO_L, LOW);
}

/* =====================================
 *  Turn Wheel Three
 * ===================================== */

void turnWheelThreeClockwise(){
  analogWrite(WHEEL_THREE_ENABLE, 200);
  digitalWrite(WHEEL_THREE_R, HIGH);
  digitalWrite(WHEEL_THREE_L, LOW);
}

void turnWheelThreeCounterClockwise(){
  analogWrite(WHEEL_THREE_ENABLE, 200);
  digitalWrite(WHEEL_THREE_R, LOW);
  digitalWrite(WHEEL_THREE_L, HIGH);
}

/* =====================================
 *  Turn Wheel Four
 * ===================================== */

void turnWheelFourClockwise(){
  analogWrite(WHEEL_FOUR_ENABLE, 200);
  digitalWrite(WHEEL_FOUR_R, HIGH);
  digitalWrite(WHEEL_FOUR_L, LOW);
}

void turnWheelFourCounterClockwise(){
  analogWrite(WHEEL_FOUR_ENABLE, 200);
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

