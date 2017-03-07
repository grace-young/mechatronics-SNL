#include <Pulse.h>

/*---------------Includes-----------------------------------*/

#include <Timers.h>

/*---------------Module Defines-----------------------------*/

#define BAUD_RATE 9600

// Stepper Motor things
#define NUM_PULSES         1
#define STEPPER_PERIOD     200

// Pins
#define OUTPUT_MOTOR_PIN   7
#define OUTPUT_DIR_PIN     6
#define OUTPUT_FLYWHEEL    9
#define PIN_POT           16

/*---------------Module Function Prototypes-----------------*/

void SetupPins(void);
void SetupTimerInterrupt(void);

/*---------------Module Variables---------------------------*/

static bool pulses_done;
static bool flywheelon;

/*---------------Lab 1 Main Functions-----------------------*/

void setup() {
  Serial.begin(BAUD_RATE);
  
  SetupPins(); 
  pulses_done = false;
  flywheelon = false;
   digitalWrite(OUTPUT_DIR_PIN, LOW);        
  SetupStepperMotor();
}

void loop() {

  if(IsPulseFinished()){
     pulses_done = true;
  }
  digitalWrite(OUTPUT_FLYWHEEL,LOW);
  /*if(flywheelon){
    digitalWrite(OUTPUT_FLYWHEEL,LOW);
  }
  else{
    digitalWrite(OUTPUT_FLYWHEEL,HIGH);
  }
  */
}

/*----------------Module Functions--------------------------*/

void SetupStepperMotor(){
    EndPulse(); // stops the old speed pulses
     
    InitPulse(OUTPUT_MOTOR_PIN, STEPPER_PERIOD); // set new pulses
    Pulse(NUM_PULSES); 
    pulses_done = false;
}

void SetupPins() {  
  pinMode(OUTPUT_FLYWHEEL, OUTPUT);
  pinMode(OUTPUT_MOTOR_PIN, OUTPUT);
  pinMode(OUTPUT_DIR_PIN, OUTPUT);
  pinMode(PIN_POT, INPUT);
}

void serialEvent(){
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // NOW, when we get a Serial event, want to start over one 
    
    if (inChar == '\n') {
       flywheelon = !flywheelon;
       if (pulses_done){
           SetupStepperMotor();
       }

    }
  }
}

