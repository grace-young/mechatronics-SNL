#include <NewPing.h>

#define TRIGGER_PIN_X 11
#define ECHO_PIN_X  10
#define TRIGGER_PIN_Y 12
#define ECHO_PIN_Y  13

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

#define MAX_DISTANCE 4000
#define ITERATIONS  5

#define PIN_OUTPUT_MOTOR_CONTROL 3
#define PIN_OUTPUT_SPEED_CONTROL 9

int bufferGyroValuesX[ITERATIONS];
int bufferGyroValuesY[ITERATIONS];
int current=0;
int check=0;
int counter=0;
long sonarXwork=MAX_DISTANCE;
long sonarYwork=MAX_DISTANCE;
int minimum=MAX_DISTANCE;
int sonarXval=MAX_DISTANCE;
int sonarYval=MAX_DISTANCE;
bool first=true;
int timePast=micros();

NewPing sonarX(TRIGGER_PIN_X, ECHO_PIN_X, MAX_DISTANCE);
NewPing sonarY(TRIGGER_PIN_Y, ECHO_PIN_Y, MAX_DISTANCE);

typedef enum{
  STATE_ROTATE, STATE_STRAFE, STATE_STOP
} States_t;
States_t state;

void setup() {
  state=STATE_ROTATE;
  pinMode(PIN_OUTPUT_MOTOR_CONTROL, OUTPUT);
  Serial.begin(115200);
  for (int i=0; i<ITERATIONS; i++){
    bufferGyroValuesX[i]=MAX_DISTANCE;
    bufferGyroValuesY[i]=MAX_DISTANCE;
  }

}

void loop() {

//delay(50);
  digitalWrite(PIN_OUTPUT_SPEED_CONTROL, COMM_MOTOR_SLOW);
  switch(state){
    case STATE_ROTATE:    
      rotateToStrafe();
      break;
    case STATE_STRAFE:
      strafeToStop();
//      analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_LEFT);
      break;
    case STATE_STOP:
      analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_STOP);
      break;
    default:
      Serial.println("FUCK");
  }
  
}

void pingAverage(){
    long x=sonarX.ping();
    if (x>50 && x<MAX_DISTANCE){
      // greater than 50 because sometimes sensors give 0
      sonarXwork=x; // old value
      bufferGyroValuesX[current]=x;
    } else {
      bufferGyroValuesX[current]=sonarXwork;
    }

    // don't need as much accuracy for y direction
    long y=sonarY.ping()/10;
    if (y>50 && y<MAX_DISTANCE){
      sonarYwork=y;
      bufferGyroValuesY[current]=y;
    } else {
      bufferGyroValuesY[current]=sonarYwork;
    }
//     bufferGyroValuesX[current]=sonarX.ping();
//     bufferGyroValuesY[current]=sonarY.ping()/10;
    int averageX=0, averageY=0;
    for (int i=0; i<ITERATIONS; i++){
      averageX=averageX+bufferGyroValuesX[i];
      averageY=averageY+bufferGyroValuesY[i];
    }
    averageX=averageX/ITERATIONS;
    sonarXval=averageX;
    averageY=averageY/ITERATIONS;
    sonarYval=averageY;
    
    current++;
    if (current>=ITERATIONS){
      current=0;
    }
  
  String toprint="counter: "+(String)counter+" sonarXval: "+(String)sonarXval +" xMin: "+(String) minimum+" y: "+y+" sonarYVal: "+(String)sonarYval+ " state: "+(String)state;
  //  Serial.println(toprint);
}


void rotateToStrafe(){
  counter++;
  if (micros()-timePast>=5){
    // want to be slight delay because clock too fast to count pings
    pingAverage(); // runs 4 times before we get *actual* value
    if (counter>=5 && counter<=8){
      // set direction of motor only once
      if (sonarYval>275){
        Serial.println("right/clockwise");
        analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CL);
        // CHANGES DIRECTIO N --> right
      } else {
        Serial.println("left/counterclockwise");
        analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_SPIN_CC);
        // CHANGES DIRECTION --> left
      }
    }
    if (sonarXval < minimum && sonarXval != 0){
      minimum = sonarXval;
      check = 0;
    } 
    if (abs(sonarXval - minimum) <= 5 && sonarXval < 500){
      //if (abs(sonarXval - minimum) >= 5 && sonarXval<500)
      // might have to play around with 5
      // maybe less than 5
      check++; 
      if (check >= 1){
        // might have to change to >= 3
        // COMM_MOTOR_STOP makes robot turn right
          counter=0; // --> for init 5 times in the beginning
          Serial.println("change state");
          analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_STOP); // stop spinning
          state=STATE_STRAFE;
          check=0;
      }
    }
    
    timePast=micros(); // this has certain time, want 5 microseconds
  }
}

void strafeToStop(){
  counter++;
  if (micros()-timePast >= 25){
    // wait longer to ping b/c y is giving more issues
    pingAverage();
    //if (counter>=5 && counter<=10){
      if (counter < 3){
        if (sonarYval > 275){
          Serial.println("right");
          analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_RIGHT);
        } else {
          Serial.println("left");
          analogWrite(PIN_OUTPUT_MOTOR_CONTROL, COMM_MOTOR_LEFT);
        }
    }
    if (sonarYval >= 265 && sonarYval <= 275){
      // test the 265 to 275 range on horz side of board
      check++;
      if (check>=3){
        Serial.println("stop!!!");
        state=STATE_STOP;
      }
    } else {
      check=0;
    }
    timePast=micros();
  }
}

