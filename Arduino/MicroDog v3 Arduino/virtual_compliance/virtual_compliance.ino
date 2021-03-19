#include <Leg3D.h>

// initialize filters (m)
float dz1_filter = 0;
float dz2_filter = 0;
float dz3_filter = 0;
float dz4_filter = 0;

// 
float tOld = 0;
//amplitude of sine wave in meters
float amp = 0.01;//.01252;
//frequency of sine wave in rad/s
float freq = 6;

//set up for the Right Front leg:
// function prototype is Leg3D(side, diagonal, hip_center, femur_center, tibia_center, hip_num, femur_num, tibia_num)
// For inboard leg setup: left - side 2, and right - side 1
// For outboard leg setup: left - side 1, and right - side 2
// For both set-ups: FL and RR hips - diagonal 1, and FR and RL hips - diagonal 2
// For both set-ups: front - face 1, and rear - face 2
//Leg3D leg1 = Leg3D(2,1,1,90,90,90,2,1,0);
//Leg3D leg2 = Leg3D(1,2,1,90,90,90,5,4,3);
//Leg3D leg3 = Leg3D(1,1,2,90,90,90,8,7,6);
//Leg3D leg4 = Leg3D(2,2,2,90,90,90,11,10,9);

Leg3D leg1 = Leg3D(1,1,1,90,90,90,2,1,0);
Leg3D leg2 = Leg3D(2,2,1,90,90,90,5,4,3);
Leg3D leg3 = Leg3D(2,1,2,90,90,90,8,7,6);
Leg3D leg4 = Leg3D(1,2,2,90,90,90,11,10,9);

void setup() {
  Serial.begin(115200);

  leg1.attach();  
  leg2.attach();  
  leg3.attach();  
  leg4.attach();  
}

void loop() {
  //get current time
  float t = millis()/1000.0;
  float dt = t - tOld;
  Serial.print(dt);
  Serial.print("\t");

  float x = 0;
  float y = 0;
  float z = 0; //amp*sin(freq*t);
  float z2 = 0; // amp*sin(freq*t);

  // Read each analog force sensor
  // My robot: [f1, f2, f3, f4] = [-314, -298, -300, -341]
  // Professor Brown's copy:
  float f1 = analogRead(0);// - 322;// - 314; //278;  // front right leg
  Serial.print(f1);
  Serial.print("\t");

  float f2 = analogRead(1);// - 257;// - 298; // 274  // front right leg
  Serial.print(f2);
  Serial.print("\t");

  float f3 = analogRead(2);// - 273;// - 300;  // 250 // front right leg
  Serial.print(f3);
  Serial.print("\t");

  float f4 = analogRead(3);// - 225;// - 341;  // 283 // front right leg
  Serial.print(f4);
  Serial.print("\t");

  // Calculate how much the foot should compensate
  float dz1 = senseForce(f1,dt);
  float dz2 = senseForce(f2,dt);
  float dz3 = senseForce(f3,dt);
  float dz4 = senseForce(f4,dt);

  // Filter each force compensation value
  dz1_filter = dz1_filter + 0.08*(dz1 - dz1_filter);
  Serial.print(dz1_filter);
  Serial.print("\t");
  dz2_filter = dz2_filter + 0.08*(dz2 - dz2_filter);
  Serial.print(dz2_filter);
  Serial.print("\t");
  dz3_filter = dz3_filter + 0.08*(dz3 - dz3_filter);
  Serial.print(dz3_filter);
  Serial.print("\t");
  dz4_filter = dz4_filter + 0.08*(dz4 - dz4_filter);
  Serial.print(dz4_filter);
  Serial.print("\t");

  // z_comp1

  // Find total compensation
  float zfr_actual = z2 + dz1_filter;
//  Serial.print(zfr_actual);
//  Serial.print("\t");
  float zfl_actual = z2 + dz2_filter;
//  Serial.print(zfl_actual);
//  Serial.print("\t");
  float zrl_actual = z + dz3_filter;
//  Serial.print(zrl_actual);
//  Serial.print("\t");
  float zrr_actual = z + dz4_filter;
//  Serial.print(zrr_actual);


  // Update leg position using Leg3D library
//  leg1.update(x,y,zfr_actual);
//  leg2.update(x,y,zfl_actual);
//  leg3.update(x,y,zrl_actual);
//  leg4.update(x,y,zrr_actual);

  leg1.update(x,y,z);
  leg2.update(x,y,z);
  leg3.update(x,y,z);
  leg4.update(x,y,z);

  tOld = t; // reset old time value
  Serial.println("");
}

float senseForce(float zFootRaw, float dt) {
  float float_to_m = 0.000007;
  float dzFoot = float_to_m * zFootRaw;
  // Calculate force experience by foot
  float kFoot = 2795; // spring constant of foot, found empirically (N/m)
  float Ffoot = kFoot * dzFoot;
  // Calculate desired displacement using virtual spring (m)
  float kVirtual = 200; // spring constant of knee joint, guessed (N/m)
  float dz = Ffoot / kVirtual;
//  dz_filter = dz_filter + (0.01 / 0.0025)*(dz - dz_filter);
//  dz_filter = dz_filter + 0.05*(dz - dz_filter);

//  Serial.print(dz);
//  Serial.print("\t");
//  Serial.print(dz_filter);
//  Serial.print("\t");

//  return dz_filter;
  return dz;
}
