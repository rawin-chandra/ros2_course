/*
 code for motor encoder testing

*/

volatile int x = 0;
volatile int y = 0;

void count() {
   x++;
}
void count2() {
   y++;
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(2), count, RISING); //channel A left motor
    attachInterrupt(digitalPinToInterrupt(19), count2, RISING); //channel A right motor
    Serial.begin(9600);
}
void loop() {
  Serial.print("x = ");
  Serial.println(x);
  Serial.print("                 y = ");
  Serial.println(y);
  delay(3);
}
