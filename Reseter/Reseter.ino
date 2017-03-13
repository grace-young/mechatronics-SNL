
void setup() {
  Serial.begin(9600);
  pinMode(8,OUTPUT);
  pinMode(A0,INPUT_PULLUP);

}

void loop() {
//  digitalWrite(8,HIGH);
//  delay(1000);
//  digitalWrite(8,LOW);
//  delay(1000);
if(!digitalRead(A0)){
  Serial.println(digitalRead(A0));
}
  
}
