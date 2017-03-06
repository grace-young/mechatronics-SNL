#include <Timers.h>
#include <Pulse.h>

#define PULSE_STEPPER 7
#define FLYWHEEL_ON A5
#define STEPPER_PERIOD 200
#define NUM_PULSES 28

#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 3000

bool shoot = true;
bool pulses_done = false;

void setup() {
  Serial.begin(9600);
  pinSetup();
  //setupPulses();
  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
}

void pinSetup() {
  pinMode(PULSE_STEPPER, OUTPUT);
  pinMode(FLYWHEEL_ON, OUTPUT);
}

void loop() {
  if(shoot){
    respondToShoot();
  }
  else{
    endShoot();
  }
}

void setupPulses(){
  Serial.println("setup");
  EndPulse(); // stops the old speed pulses 
  InitPulse(PULSE_STEPPER, STEPPER_PERIOD); // set new pulses
  Pulse(NUM_PULSES); 
  pulses_done = false;
  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
}

void endShoot(){
  digitalWrite(FLYWHEEL_ON, LOW);
  EndPulse();
}

void respondToShoot() {
    digitalWrite(FLYWHEEL_ON, HIGH);
    if(IsPulseFinished()){
      pulses_done = true;
    }
    if(TMRArd_IsTimerExpired(STEPPER_WAIT_TIMER)){
      setupPulses();
    }
}

