/* Step and Sway
 *  February 18, 2021
 *  1. Performing a step change in y-direction of the body (or the feet).
 *  2. Performing swaying of body
 *  Goal: Use the optical flow sensor the measure motion, 
 *  and determine the servo dynamics and CG position
 */

#include <Leg3D.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Bitcraze_PMW3901.h"
#include <VL53L1X.h>


// Using digital pin 12 for chip select
Bitcraze_PMW3901 flow(12);

VL53L1X sensor;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


Leg3D leg1 = Leg3D(1,1,1,90,90,90,2,1,0);
Leg3D leg2 = Leg3D(2,2,1,90,90,90,5,4,3);
Leg3D leg3 = Leg3D(2,1,2,90,90,90,8,7,6);
Leg3D leg4 = Leg3D(1,2,2,90,90,90,11,10,9);

float xfl; 
float yfl;
float zfl;
float xfr;
float yfr;
float zfr;
float xrl;
float yrl;
float zrl;
float xrr;
float yrr;
float zrr;

float startTime;
float dT;
float oldT = millis();
float totalDisplace = 0;
float disX = 0;
float disY = 0;
int16_t deltaX,deltaY;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the IMU sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  
  // Initialize the PMW3901 Optical Flow sensor
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }

//  // Initialize VL53L1X Time of Flight sensor
//  sensor.setTimeout(500);
//  if (!sensor.init())
//  {
//    Serial.println("Failed to detect and initialize sensor!");
//    while (1);
//  }
//  
//  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
//  // You can change these settings to adjust the performance of the sensor, but
//  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
//  // medium and long distance modes. See the VL53L1X datasheet for more
//  // information on range and timing limits.
//  sensor.setDistanceMode(VL53L1X::Long);
//  sensor.setMeasurementTimingBudget(50000);
//
//  // Start continuous readings at a rate of one measurement every 50 ms (the
//  // inter-measurement period). This period should be at least as long as the
//  // timing budget.
//  sensor.startContinuous(50);

  leg1.attach();  
  leg2.attach();  
  leg3.attach();  
  leg4.attach(); 

  startTime = millis();
}

void loop() {
  dT = millis() - oldT;
  oldT = millis();
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY);

  // Filter using threshold reading value to prevent drift in position
//  if(abs(deltaX) > 4) {
//    disX = disX + deltaX*dT;
//  }
//  if(abs(deltaY) > 4) {
//    disY = disY + deltaY*dT;
//  }

  // Without filtering
  disX = disX + deltaX*dT;
  disY = disY + deltaY*dT;

  Serial.print(millis());
  Serial.print("\t");
//  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print("\t");
//  Serial.print(", Y: ");
  Serial.print(deltaY);
  Serial.print("\t");
//  Serial.print(", Dis X: ");
  Serial.print(disX);
  Serial.print("\t");
//  Serial.print(", Dis Y: ");
  Serial.print(disY);
  Serial.print("\n");
  
//  // VL53L1X
//  Serial.print(sensor.read());
//  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
//
//  Serial.println();

  if((millis() - startTime) < 5000) {
    xfl = 0; 
    yfl = 0;
    zfl = 0;
    xfr = 0;
    yfr = 0;
    zfr = 0;
    xrl = 0;
    yrl = 0;
    zrl = 0;
    xrr = 0;
    yrr = 0;
    zrr = 0;
//    Serial.println("Let's...");  // debugging
  }
  else {
    xfl = 0; 
    yfl = 0.0254;
    zfl = 0;
    xfr = 0;
    yfr = -0.0254;
    zfr = 0;
    xrl = 0;
    yrl = 0.0254;
    zrl = 0;
    xrr = 0;
    yrr = -0.0254;
    zrr = 0;
//    Serial.println("Go!");  // debugging
  }

  

  // Update Leg positions
  leg1.update(xfr,yfr,zfr);
  leg2.update(xfl,yfl,zfl);
  leg3.update(xrl,yrl,zrl);
  leg4.update(xrr,yrr,zrr);
  

}
