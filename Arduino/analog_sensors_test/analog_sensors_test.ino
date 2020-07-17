void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(5,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(5,HIGH);
  digitalWrite(11,HIGH);
}

void loop() {
  Serial.print(analogRead(0));
  Serial.print("\t");
  Serial.print(analogRead(1));
  Serial.print("\t");
  Serial.print(analogRead(2));
  Serial.print("\t");
  Serial.print(analogRead(3));
  Serial.print("\t");
  Serial.print(analogRead(4));
  Serial.print("\t");
  Serial.print(analogRead(5));
  Serial.println("\t");
  delay(10);
  // put your main code here, to run repeatedly:

}
