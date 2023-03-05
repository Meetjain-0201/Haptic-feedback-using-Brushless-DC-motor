#include <Servo.h>

Servo ESC1;     // create servo object to control the ESC
Servo ESC2;

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(PB6,10000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(PB7,10000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  Serial.begin(9600);
}

void loop() {
  potValue = analogRead(A5);   // reads the value of the potentiometer (value between 0 and 1023)
  int reading1 = analogRead(A0);
  Serial.print(reading1);
  Serial.print(" ");
  int reading2 = analogRead(A1);
  Serial.print(reading2);
  Serial.print(" ");
  Serial.println(potValue);
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC1.write(potValue);    // Send the signal to the ESC
  ESC2.write(potValue);    // Send the signal to the ESC
}
