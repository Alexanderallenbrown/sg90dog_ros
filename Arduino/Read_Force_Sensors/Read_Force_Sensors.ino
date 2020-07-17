#include <Wire.h>


int Address = 100;

bool receiveFlag = false;
bool sendFlag = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Wire.begin(Address);
  Wire.onRequest(requestEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(analogRead(0));  // front right leg
  Serial.print("\t");
  Serial.print(analogRead(1));  // front left leg
  Serial.print("\t");
  Serial.print(analogRead(2));  // rear left leg
  Serial.print("\t");
  Serial.print(analogRead(3));  // rear right leg
  Serial.print("\t");
  //byte val1 = byte(map(analogRead(0)-(279),0,1023 - 279,0,255));  //750  // front right leg
  //Serial.print(val1);
  Serial.println();
  delay(1);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent(){
  int offset1 = 279 - 100;
  int offset2 = 327 - 100;
  int offset3 = 294 - 100;
  int offset4 = 288 - 100;
  byte val1 = byte(map(analogRead(0)-(offset1),0,1023 - offset1,0,255));  //750  // front right leg
  byte val2 = byte(map(analogRead(1)-(offset2),0,1023 - offset2,0,255));  // front left leg
  byte val3 = byte(map(analogRead(2)-(offset3),0,1023 - offset3,0,255));  // rear left leg
  byte val4 = byte(map(analogRead(3)-(offset4),0,1023 - offset4,0,255));  // rear right leg
//  Serial.println(val1);
  Wire.write(val1);
  Wire.write(val2);
  Wire.write(val3);
  Wire.write(val4);
//  Wire.endTransmission();

  //sendFlag = true;
}
