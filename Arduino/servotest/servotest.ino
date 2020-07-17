/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

float amp = 0; //degrees
float freq = 3; //rad/s

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Microdog Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}



void loop() {

  //calculate current time
  float t = millis()/1000.0;
  //calculate servo position in degrees;
  float pos = amp*sin(t*freq)+90;

  //convert this position to a pulse
//  float fpulse = (SERVOMAX-SERVOMIN)*1.0/180.0*pos + SERVOMIN;
float fpulse = (USMAX-USMIN)*1.0/180.0*pos + USMIN;

  uint16_t pulselen = int(fpulse);

  for(int servonum=0;servonum<12;servonum++){
//    pwm.setPWM(servonum, 0, pulselen);
pwm.writeMicroseconds(servonum, pulselen);
  }

 Serial.print(pwm.getPWM(4));
 Serial.print("\t");
 Serial.print(pos);
 Serial.print("\t");
 Serial.println(pulselen);

delay(10);
}
