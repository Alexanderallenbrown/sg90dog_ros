#include <Servo.h>

Servo myservo;  // create servo object to control a servo


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myservo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  int fval = analogRead(2);
  int dx = (fval-699);

  myservo.write(90+dx);


  
Serial.println(analogRead(0));
delay(1);
}
