#include <Timers.h>
#include <Pulse.h>
#include <Servo.h>

//STEPPER
#define PIN_OUT_PULSE_STEPPER 5
#define PIN_OUT_FLYWHEEL_ON 6
#define PIN_IN_START_SHOOTING 7
#define PIN_OUT_SHOOTING_END 8
#define PIN_OUT_PADDLE      11

#define STEPPER_PERIOD 200

#define STEPPER_WAIT_TIMER 0
#define STEPPER_TIME_INTERVAL 700

//SERVO
#define SERVO_PIN_PULSE_OUT 9
#define SERVO_PULSE_IN 10

//STEPPER

static bool shoot = true;
static bool pulses_done = false;
static bool sawShoot = false;
static bool calibrate_tick = true;
static bool paddleUp = false;

static int NUM_PULSES  = 29;
static int numShot = 0;
const int CALIBRATE_COUNT = 3;
static int numToShoot = 100;

static int count = 1;
static int shoot_count;

//SERVO
const int servo_angle = 165;
static bool turn_servo = true;

Servo myservo;
Servo paddleServo;

void setup() {
  Serial.begin(9600);
  pinSetup();
  //setupPulses();
  TMRArd_InitTimer(STEPPER_WAIT_TIMER, STEPPER_TIME_INTERVAL);
  digitalWrite(PIN_OUT_SHOOTING_END, LOW);
  digitalWrite(PIN_OUT_FLYWHEEL_ON,LOW);
  digitalWrite(PIN_OUT_PULSE_STEPPER,LOW);

  myservo.attach(SERVO_PIN_PULSE_OUT);
  myservo.write(servo_angle);
  paddleServo.attach(PIN_OUT_PADDLE);
  paddleServo.write(0);
  retractEye();
}

void pinSetup() {
  pinMode(PIN_OUT_PULSE_STEPPER, OUTPUT);
  pinMode(PIN_OUT_FLYWHEEL_ON, OUTPUT);
  pinMode(PIN_OUT_SHOOTING_END, OUTPUT);
  pinMode(PIN_IN_START_SHOOTING, INPUT);
  pinMode(SERVO_PIN_PULSE_OUT, OUTPUT);
  pinMode(SERVO_PULSE_IN, INPUT);
}

void loop() {
  armEye();
  if(shouldShoot() && !sawShoot){
    shoot = true;
    announceStillShooting();
    sawShoot = true;
  }
  if(numShot >= numToShoot) {
    endShoot();
    delay(1000); //is this what we want?
  }
  else if(shoot) {
    announceStillShooting();
    respondToShoot();
  }

  if (shouldDeployEye() && turn_servo) {
    armEye();
    //Serial.println("ARMED");
  } else if (!shouldDeployEye() && !turn_servo) {
    retractEye();
    //Serial.println("RETRACTED");
  } 
}

bool shouldDeployEye(){
  return digitalRead(SERVO_PULSE_IN);
}

bool shouldShoot(){
  return digitalRead(PIN_IN_START_SHOOTING);
}

void announceStillShooting(){
  digitalWrite(PIN_OUT_SHOOTING_END,HIGH);
}

void setupPulses(){
  numShot++;
//  Serial.println("setup");
  EndPulse(); // stops the old speed pulses 
  InitPulse(PIN_OUT_PULSE_STEPPER, STEPPER_PERIOD); // set new pulses
  count++;
  shoot_count++;
  if (count == CALIBRATE_COUNT && calibrate_tick) {
    calibrate_tick = false;
    count = 1;
    NUM_PULSES--;
  } else if (count == CALIBRATE_COUNT && !calibrate_tick) {
    calibrate_tick = true;
    count = 1;
    NUM_PULSES++;
  }
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
  digitalWrite(PIN_OUT_SHOOTING_END, LOW);
}

void respondToShoot() {
    digitalWrite(PIN_OUT_FLYWHEEL_ON, HIGH);
    delay(250);
    if(IsPulseFinished()){
      pulses_done = true;
    }
    if(TMRArd_IsTimerExpired(STEPPER_WAIT_TIMER)){
      setupPulses();
    }
}

void armEye() {
   myservo.write(servo_angle);
   turn_servo = false;
   if(!paddleUp){
    paddleServo.write(90);
   }
   paddleUp = true;
}

void retractEye() {
  myservo.write(-servo_angle);
  turn_servo = true;
}
