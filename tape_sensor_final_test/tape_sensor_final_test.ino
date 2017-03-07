#define PIN_SENSOR_A 7 
#define PIN_SENSOR_B_1 5
#define PIN_SENSOR_C 8
#define PIN_SENSOR_D_1 13
#define PIN_SENSOR_E 12

// need pins for these:
#define PIN_SENSOR_B_2 6
#define PIN_SENSOR_D_2 11

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  SetupPins();

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("A ");
  Serial.print(ReadTapeSensor_A());
  Serial.print(" B1 ");
  Serial.print(ReadTapeSensor_B_1());
  Serial.print(" B2 ");
  Serial.print(ReadTapeSensor_B_2());
  Serial.print(" C ");
  Serial.print(ReadTapeSensor_C());
  Serial.print(" D1 ");
  Serial.print(ReadTapeSensor_D_1());
  Serial.print(" D2 ");
  Serial.print(ReadTapeSensor_D_2());
  Serial.print(" E ");
  Serial.println(ReadTapeSensor_E());
}

/* ====================================================================
 *  ReadTapeSensor functions
 * ====================================================================
 */

bool ReadTapeSensor_A(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_A);
}

bool ReadTapeSensor_B_1(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_B_1);
}

bool ReadTapeSensor_B_2(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_B_2);
}

bool ReadTapeSensor_C(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_C);
}

bool ReadTapeSensor_D_1(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_D_1);
}

bool ReadTapeSensor_D_2(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_D_2);
}

bool ReadTapeSensor_E(void){
    // 1 if read tape, 0 if not 
    return !digitalRead(PIN_SENSOR_E);
}

void SetupPins(){
  pinMode(PIN_SENSOR_A, INPUT);
  pinMode(PIN_SENSOR_B_1, INPUT);
  pinMode(PIN_SENSOR_B_2, INPUT);
  pinMode(PIN_SENSOR_C, INPUT);
  pinMode(PIN_SENSOR_D_1, INPUT);
  pinMode(PIN_SENSOR_D_2, INPUT);
  pinMode(PIN_SENSOR_E, INPUT);
}
