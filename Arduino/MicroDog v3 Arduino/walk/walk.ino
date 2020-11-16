#include <Leg3D.h>

//amplitude of sine wave in meters
//CHANGE THESE BEFORE USING!!!!!!!!!!!!
float xamp = 1; //0.01252; //0.017;
float zamp = 1;//.01252;
//frequency of sine wave in rad/s
float freq = 4;


//set up for the Right Front leg:
//function prototype is Leg3d(side, diagonal, hip_center, femur_center, tibia_center, hip_num, femur_num, tibia_num)
// For inboard leg setup: left - side 2, and right - side 1
// For outboard leg setup: left - side 1, and right - side 2
// For both set-ups: FL and RR hips - diagonal 1, and FR and RL hips - diagonal 2
// For both set-ups: front - face 1, and rear - face 2
Leg3D leg1 = Leg3D(2,1,1,90,90,90,2,1,0);
Leg3D leg2 = Leg3D(1,2,1,90,90,90,5,4,3);
Leg3D leg3 = Leg3D(1,1,2,90,90,90,8,7,6);
Leg3D leg4 = Leg3D(2,2,2,90,90,90,11,10,9);

void setup(){
  Serial.begin(115200);
  leg1.attach();  
  leg2.attach();  
  leg3.attach();  
  leg4.attach();  
}

void loop(){
    //get current time
    float t = millis()/1000.0;

//    float x = 0;
//    float y = 0;
//    float z = amp*sin(freq*t);

    float phifr = 0;
    float phirl = 3.14159265/2;
    float phifl = 3.14159265;
    float phirr = 3*3.14159265/2;
    float xfl = xamp*sin(freq*t+phifl);
    float yfl = 0;
    float zfl = zamp*cos(freq*t+phifl);
    float xfr = xamp*sin(freq*t+phifr);
    float yfr = 0;
    float zfr = zamp*cos(freq*t+phifr);
    float xrl = xamp*sin(freq*t+phirl);
    float yrl = 0;
    float zrl = zamp*cos(freq*t+phirl);
    float xrr = xamp*sin(freq*t+phirr);
    float yrr = 0;
    float zrr = zamp*cos(freq*t+phirr);

    if(zfr < 0) {
      zfr=0;
    }
    if(zfl < 0) {
      zfl=0;
    }
    if(zrl < 0) {
      zrl=0;
    }
    if(zrr < 0) {
      zrr=0;
    }

    leg1.update(xfr,yfr,zfr);
    leg2.update(xfl,yfl,zfl);
    leg3.update(xrl,yrl,zrl);
    leg4.update(xrr,yrr,zrr);

    Serial.print(xfl);
    Serial.print("\t");
    Serial.print(yfl);
    Serial.print("\t");
    Serial.print(zfl);
    Serial.println();
    delay(5);
}
