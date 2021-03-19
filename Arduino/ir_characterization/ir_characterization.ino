/* IR Sensor Characterization
 *  Gabrielle G. Conard
 *  March 13, 2021
 *  Goal: Characterize/callibrate the infrared (IR) sensors
 */

#include <Leg3D.h>

//function prototype is Leg3d(side, diagonal, face, hip_center, femur_center, tibia_center, hip_num, femur_num, tibia_num)
Leg3D leg1 = Leg3D(1,1,1,90,90,90,2,1,0);  // front right
Leg3D leg2 = Leg3D(2,2,1,90,90,90,5,4,3);  // front left
Leg3D leg3 = Leg3D(2,1,2,90,90,90,8,7,6);  // rear left
Leg3D leg4 = Leg3D(1,2,2,90,90,90,11,10,9);  // rear right

float irReading;  // reading from IR sensor (counts)
float dist;  // distance away from sensor

void setup() {
    // Attach all legs
    leg1.attach();  
    leg2.attach();  
    leg3.attach();  
    leg4.attach(); 
  
    // Turn on emitter diode
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    
    Serial.begin(9600); // set baud rate
}

void loop() {
    stand();  // make robot stand up
  
    irReading = analogRead(A4);  // read receiving diode
  
    dist = 37.422*pow(irReading,-0.585); // distance in inches
//    dist = 0.9505*pow(irReading,-0.585); // distance in meters
  
    // print to Serial Monitor
    Serial.print(irReading);  
    Serial.print('\t');
    Serial.println(dist);
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
