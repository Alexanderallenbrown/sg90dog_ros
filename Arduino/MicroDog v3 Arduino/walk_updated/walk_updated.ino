#include <Leg3D.h>

float pi = 3.14159265;

//amplitude of sine wave in meters
float xamp = 0.01252; //0.017;
float zamp = .01252;
//float xamp = 1.0; //0.017;
//float zamp = 1.0;
//frequency of sine wave in rad/s
float freq = 4.0;

unsigned long t = millis();

int n;  // leg number, 1-4 (fl, rr, fr, rl)
float Wgait = 2*pi;  // walking gait angular velocity (rad/s)
float swingThres;  // phase angle threshold before entering swing  (rad)
float swingDur;    // duration of swing cycle  (rad)
float phi;         // phase shift (rad)
float x;
float z;
float t1;
String phase;

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

//    float x = 0;
//    float y = 0;
//    float z = amp*sin(freq*t);

    float t = millis()/1000.0;
//    float phifr = 0;
//    float phirl = 3.14159265/2;
    float phifl = 3.14159265;
//    float phirr = 3*3.14159265/2;
//    float xfl = xamp*sin(freq*t+phifl);
//    float yfl = 0;
//    float zfl = zamp*cos(freq*t+phifl);
//    float xfr = xamp*sin(freq*t+phifr);
//    float yfr = 0;
//    float zfr = zamp*cos(freq*t+phifr);
//    float xrl = xamp*sin(freq*t+phirl);
//    float yrl = 0;
//    float zrl = zamp*cos(freq*t+phirl);
//    float xrr = xamp*sin(freq*t+phirr);
//    float yrr = 0;
//    float zrr = zamp*cos(freq*t+phirr);

    // Walk
//    float xfl = xamp*xGait(1); 
//    float yfl = 0;
//    float zfl = zamp*zGait(1);
////    float xfr = xamp*xGait(3);
////    float yfr = 0;
////    float zfr = zamp*zGait(3);
//    float xrl = xamp*xGait(4);
//    float yrl = 0;
//    float zrl = zamp*zGait(4);
//    float xrr = xamp*xGait(2);
//    float yrr = 0;
//    float zrr = zamp*zGait(2);
//
//    float xfr = 0;
//    float yfr = -0.01;//*sin(freq*t+phifl);
//    float zfr = zamp*zGait(3);


    // Stand
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

//    Serial.print(t1);
//    Serial.print("\t");
    Serial.print(xfl);
    Serial.print("\t");
    Serial.print(yfl);
    Serial.print("\t");
    Serial.print(zfl);
    Serial.print("\t");
//    Serial.print(phase);
    Serial.println();
//    delay(5);
}

float xGait(int n) {
    swingThres = 3*pi/2 + n*pi/2;
    swingDur = pi/2;
    phi = pi/2;


    t1 = timer()*1.0e-3;
    

    if(swingThres >= 2*pi) {
      swingThres = swingThres - 2*pi;
    }

    if((Wgait*t1 > swingThres) && (Wgait*t1 < (swingThres + swingDur))) {
      x = cos(Wgait * 2.0 * t1 - (phi*n)*2.0);
      phase = "fast";
    }
    else if(Wgait*t1 <= swingThres) {
      x = cos(Wgait*(2.0/3.0)*t1- ((phi*n+pi))*(2.0/3.0));
      phase = "slow 1";
    }
    else {
      x = cos(Wgait*(2.0/3.0)*t1 - (phi*n)*(2.0/3.0));
      phase = "slow 2";
    }

    return x;
}



float zGait(int n) {
    t1 = t1 * 1e-3;
    swingThres =  2*pi + (n-1)*pi/2;
    swingDur = pi/2;
    phi = pi/2;


    t1 = timer()*1.0e-3;
//    Serial.println(t1);
    
    if(swingThres >= 2*pi) {
      swingThres = swingThres - 2*pi;
    }

    if((Wgait*t1 > swingThres) && (Wgait*t1 < (swingThres + swingDur))) {
      z = sin(Wgait * 2.0 * t1 - (phi*(n+3))*2.0);
    }
    else if(Wgait*t1 <= swingThres) {
      z = sin(Wgait*(2.0/3.0)*t1 - ((phi*(n+3)+pi))*(2.0/3.0));
    }
    else {
      z = sin(Wgait*(2.0/3.0)*t1 - (phi*(n+3))*(2.0/3.0));
    }

    return z;
}
