#include <NewPing.h>

#define TRIGGER_PIN_X 11
#define ECHO_PIN_X  10
#define TRIGGER_PIN_Y 12
#define ECHO_PIN_Y  13

#define MAX_DISTANCE 4000
#define ITERATIONS  5

#define output 3

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
  
  pinMode(output, OUTPUT);
  Serial.begin(115200);
  for (int i=0; i<ITERATIONS; i++){
    bufferGyroValuesX[i]=MAX_DISTANCE;
    bufferGyroValuesY[i]=MAX_DISTANCE;
  }

}

void loop() {

delay(50);

  switch(state){
    case STATE_ROTATE:
    
      
      rotateToStrafe();
      
      break;
    case STATE_STRAFE:
      strafeToStop();
//      analogWrite(output, 180);
      break;
    case STATE_STOP:
       
      analogWrite(output, 160);
      break;
    default:
      Serial.println("FUCK");
  }
  
}

void pingAverage(){
    long x=sonarX.ping();
    if (x>50 && x<MAX_DISTANCE){
      sonarXwork=x;
      bufferGyroValuesX[current]=x;
    } else {
      bufferGyroValuesX[current]=sonarXwork;
    }

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
    Serial.println(toprint);
}
void rotateToStrafe(){
  counter++;
  if (micros()-timePast>=5){
    
    pingAverage();
    if (counter>=5 && counter<=8){
      if (sonarYval>275){
        Serial.println("right/clockwise");
        analogWrite(output, 140);
      } else {
        Serial.println("left/counterclockwise");
        analogWrite(output, 120);
      }
    }
    if (sonarXval<minimum && sonarXval !=0){
      minimum=sonarXval;
      check=0;
    } 
    if (sonarXval-minimum>=5 && sonarXval<500){
      check++;
      if (check>=1){
        
//        analogWrite(output, 160);
        counter=0;
          Serial.println("change state");
          state=STATE_STRAFE;
          check=0;
      }
    }
    
    timePast=micros();
  }
}
void strafeToStop(){
  counter++;
  if (micros()-timePast>=25){
    pingAverage();
    if (counter>=5 && counter<=10){
      if (sonarYval>275){
        Serial.println("right");
        analogWrite(output, 100);
      } else {
        Serial.println("left");
        analogWrite(output, 180);
      }
    }
    if (sonarYval>=265 && sonarYval<=275){
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

