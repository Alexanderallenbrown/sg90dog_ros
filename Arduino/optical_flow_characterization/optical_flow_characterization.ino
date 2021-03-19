/* Optical Flow Sensor Characterization
 *  Gabrielle G. Conard
 *  March 14, 2021
 *  Goal: Characterize/callibrate the optical flow sensor
 */


#include <Leg3D.h>
#include "Bitcraze_PMW3901.h"
#include <VL53L1X.h>

//function prototype is Leg3d(side, diagonal, face, hip_center, femur_center, tibia_center, hip_num, femur_num, tibia_num)
Leg3D leg1 = Leg3D(1,1,1,90,90,90,2,1,0);  // front right
Leg3D leg2 = Leg3D(2,2,1,90,90,90,5,4,3);  // front left
Leg3D leg3 = Leg3D(2,1,2,90,90,90,8,7,6);  // rear left
Leg3D leg4 = Leg3D(1,2,2,90,90,90,11,10,9);  // rear right


// Using digital pin 12 for chip select
Bitcraze_PMW3901 flow(12);

VL53L1X sensor;

int16_t deltaX,deltaY, xTotal, yTotal;
float xConvert,yConvert;

void setup() {
    // Attach all legs
    leg1.attach();  
    leg2.attach();  
    leg3.attach();  
    leg4.attach(); 

    // Initialize the PMW3901 Optical Flow sensor
    if (!flow.begin()) {
      Serial.println("Initialization of the flow sensor failed");
      while(1) { }
    }
    
    Serial.begin(9600); // set baud rate
}

void loop() {
    stand();  // make robot stand up

//    if(delayReadingTimer){
//        // Get motion count since last call
//        flow.readMotionCount(&deltaX, &deltaY);
//
//        xTotal += deltaX;
//        yTotal += deltaY;
//    
//        xConvert = xTotal*0.032081;
//        yConvert = yTotal*0.032081;
//    }

    // Get motion count since last call
    flow.readMotionCount(&deltaX, &deltaY);

    xTotal += deltaX;
    yTotal += deltaY;

//    xConvert = xTotal*0.032081;
//    yConvert = yTotal*0.032081;

    // White Letter/Black Background Paper
//    xConvert = xTotal*0.00983;
    yConvert = yTotal*0.00983;
    
    // print to Serial Monitor
//    Serial.print();  
//    Serial.print('\t');
//    Serial.println();

    Serial.print(millis());
    Serial.print("\t");
  //  Serial.print("X: ");
    Serial.print(deltaX);
    Serial.print("\t");
  //  Serial.print(", Y: ");
    Serial.print(deltaY);
    Serial.print("\t");
    Serial.print(xTotal);
    Serial.print("\t");
    Serial.print(yTotal);
//    Serial.print("\t");
//    Serial.print(xConvert);
    Serial.print("\t");
    Serial.print(yConvert);
    Serial.println();

}

void stand() {
    // Foot positions
    float xfl = 0; 
    float yfl = 0;
    float zfl = 0;
    float xfr = 0;
    float yfr = 0;
    float zfr = 0;
    float xrl = 0;
    float yrl = 0;
    float zrl = 0;
    float xrr = 0;
    float yrr = 0;
    float zrr = 0;

    // Update servo positions using Leg3D library
    leg1.update(xfr,yfr,zfr);
    leg2.update(xfl,yfl,zfl);
    leg3.update(xrl,yrl,zrl);
    leg4.update(xrr,yrr,zrr);
    
}
