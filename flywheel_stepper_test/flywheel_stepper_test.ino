#include <Timers.h>
#include <Pulse.h>

#define PIN_OUT_PULSE_STEPPER 5
#define PIN_OUT_FLYWHEEL_ON 6
#define PIN_IN_START_SHOOTING 7
#define PIN_OUT_SHOOTING_END 8

#define STEPPER_PERIOD 200
#define NUM_PULSES 28

#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 3000

static bool shoot = false;
static bool pulses_done = false;
static bool sawShoot = false;
static int numShot = 0;



void setup() {
  Serial.begin(9600);
  pinSetup();
  //setupPulses();
  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
  digitalWrite(PIN_OUT_SHOOTING_END, LOW);
  digitalWrite(PIN_OUT_FLYWHEEL_ON,LOW);
  digitalWrite(PIN_OUT_PULSE_STEPPER,LOW);
}

void pinSetup() {
  pinMode(PIN_OUT_PULSE_STEPPER, OUTPUT);
  pinMode(PIN_OUT_FLYWHEEL_ON, OUTPUT);
  pinMode(PIN_OUT_SHOOTING_END, OUTPUT);
  pinMode(PIN_IN_START_SHOOTING, INPUT);
}

void loop() {
  Serial.print("PIN");
  Serial.println(digitalRead(PIN_IN_START_SHOOTING));
  Serial.print("Shoot");
  Serial.println(shoot);
   Serial.print("numShoot");
  Serial.println(numShot);
  Serial.print("sawShoot");
  Serial.println(sawShoot);
  
  //Serial.println(
  //Serial.println(digitalRead(PIN_IN_START_SHOOTING));
  if(digitalRead(PIN_IN_START_SHOOTING) && !sawShoot){
    shoot = true;
    sawShoot = true;
  }
  if(numShot > 3){
    endShoot();
    delay(1000);
  }
  else if(shoot){
    digitalWrite(PIN_OUT_SHOOTING_END,LOW);
    respondToShoot();
  }
 
}

void setupPulses(){
  numShot++;
  Serial.println("setup");
  EndPulse(); // stops the old speed pulses 
  InitPulse(PIN_OUT_PULSE_STEPPER, STEPPER_PERIOD); // set new pulses
  Pulse(NUM_PULSES); 
  pulses_done = false;
  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
}

void endShoot(){
  numShot = 0;
  shoot = false;
  digitalWrite(PIN_OUT_FLYWHEEL_ON, LOW);
  EndPulse();
  sawShoot = false;
  digitalWrite(PIN_OUT_SHOOTING_END, HIGH);
}

void respondToShoot() {
    digitalWrite(PIN_OUT_FLYWHEEL_ON, HIGH);
    if(IsPulseFinished()){
      pulses_done = true;
    }
    if(TMRArd_IsTimerExpired(STEPPER_WAIT_TIMER)){
      setupPulses();
    }
}

