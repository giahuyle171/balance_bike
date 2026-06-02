
#include <Servo.h>
 
// Servo servo;

void log(int s) {
  if (s=="\n") Serial.println("");
  Serial.print(s);
  Serial.print(" ");
}
void setup() {
  Serial.begin(9600);
  // servo.attach(9);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
}
void loop() {
  int sw1 = digitalRead(5);
  int sw2 = digitalRead(6);
  int vol1 = analogRead(A0);
  int vol2 = analogRead(A1);

  int joy1 = analogRead(A3);
  int joy2 = analogRead(A5);
  log(0);
  log(1023);
  log(joy1);
  log(joy2);
  log(vol1);
  log(vol2);
  log(sw1);
  log(sw2);
  log("\n");
  delay(50);
}