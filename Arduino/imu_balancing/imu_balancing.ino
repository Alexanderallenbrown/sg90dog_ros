/* Balancing MicroDog
 *  February 7, 2021
 *  Based on sensorapi from the Adafruit BNO055 library
 */

#include <Leg3D.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


Leg3D leg1 = Leg3D(1,1,1,90,90,90,2,1,0);
Leg3D leg2 = Leg3D(2,2,1,90,90,90,5,4,3);
Leg3D leg3 = Leg3D(2,1,2,90,90,90,8,7,6);
Leg3D leg4 = Leg3D(1,2,2,90,90,90,11,10,9);


float tOld = 0;
//amplitude of sine wave in meters
float amp = 0.01;//.01252;
//frequency of sine wave in rad/s
float freq = 6;

const float L = 5.5*0.0254;  // length of robot chassis (m)
const float T = 3.5*0.0254;  // width of robot chassis (m)

float roll;
float pitch;
float yaw;

void setup() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  
  leg1.attach();  
  leg2.attach();  
  leg3.attach();  
  leg4.attach(); 

}

void loop() {
  //get current time
  float t = millis()/1000.0;
  float dt = t - tOld;
  tOld = t; // reset old time value

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  roll = event.orientation.z * PI/180;
  pitch = event.orientation.y * PI/180;
  yaw = event.orientation.x * PI/180;

  float x = 0; 
  float y = 0;
  
//  float zfr_actual = -(pitch*(L/2)) - roll*(T/2);
//  float zfl_actual = -(pitch*(L/2)) + roll*(T/2);
//  float zrl_actual = (pitch*(L/2)) + roll*(T/2);
//  float zrr_actual = (pitch*(L/2)) - roll*(T/2);

  float zfr_actual = (pitch*(L/2)) + roll*(T/2);
  float zfl_actual = (pitch*(L/2)) - roll*(T/2);
  float zrl_actual = -(pitch*(L/2)) - roll*(T/2);
  float zrr_actual = -(pitch*(L/2)) + roll*(T/2);

  Serial.print("\tzfr: ");
  Serial.print(zfr_actual);
  Serial.print("\tzfl: ");
  Serial.print(zfl_actual);
  Serial.print("\tzrl: ");
  Serial.print(zrl_actual);
  Serial.print("\tzrr: ");
  Serial.print(zrr_actual);

  // Update leg position using Leg3D library
  leg1.update(x,y,zfr_actual);
  leg2.update(x,y,zfl_actual);
  leg3.update(x,y,zrl_actual);
  leg4.update(x,y,zrr_actual);

  
  Serial.println("");
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
