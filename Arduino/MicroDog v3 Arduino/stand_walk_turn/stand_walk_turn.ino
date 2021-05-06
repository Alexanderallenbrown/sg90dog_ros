#include <Leg3D.h>

float pi = PI; //3.14159265;

//amplitude of sine wave in meters
float xamp = 0.0254; //0.01252; //0.017;
float yamp = 0.01252;
float zamp = 0.019; //.01252;
//float xamp = 1.0; //0.017;
//float yamp = 1.0;
//float zamp = 1.0;
//frequency of sine wave in rad/s
float freq = 4.0;

unsigned long t = millis();
float tOld = millis();
float dT;


int n;  // leg number, 1-4 (fl, rr, fr, rl)
float Wgait = 2*pi;  // walking gait angular velocity (rad/s)
float Tgait = 2*pi / Wgait;
float Tstance = (Tgait)*3/4;
float vStance = 2*xamp / (Tstance);
float swingThres;  // phase angle threshold before entering swing  (rad)
float swingDur;    // duration of swing cycle  (rad)
float phi;         // phase shift (rad)
float x;
float xOld[4]; // = {0,0,0,0};
float y;
float z;
float t1;
String phase;
String state;

// Finite State Machine Transitions and States
bool Fwd = true;
bool Back = false;
bool LeftTurn = false;
bool RightTurn = false;
bool Lf;
bool Tfb;
bool Lb;
//bool Tfb;
bool Tbr;
bool Tbl;
bool Lr;
bool Ll;
bool Trb;
bool Tlb;
bool Trf;
bool Tlf;


//bool LeftTurn = true;
//bool RightTurn = false;
//bool Tlr;
//bool Ll;
//bool Trl;
//bool Lr;
//bool cycleTime = false;

bool Go = true;
bool Stay = false;
bool Tsg;
bool Tgs;
bool Ls;
bool Lg;

// IR Sensor and Edge Detecting
float irReading;
bool edgeDetected;

bool backTime = false;
bool turnTime = false;

// Initialize buttons
int btn_C;
int btn_C_pin = 7;
int btn_C_old = 0;
int btn_C_press;

int btn_B;
int btn_B_pin = 11;
int btn_B_old = 0;
int btn_B_press;

int btn_A;
int btn_A_pin = 12;
int btn_A_old = 0;
int btn_A_press;

//set up for the Right Front leg:
//function prototype is Leg3d(side, diagonal, face, hip_center, femur_center, tibia_center, hip_num, femur_num, tibia_num)
// For inboard leg setup: left - side 2, and right - side 1
// For outboard leg setup: left - side 1, and right - side 2
// For both set-ups: FL and RR hips - diagonal 1, and FR and RL hips - diagonal 2
// For both set-ups: front - face 1, and rear - face 2
//Leg3D leg1 = Leg3D(2,1,1,90,90,90,2,1,0);
//Leg3D leg2 = Leg3D(1,2,1,90,90,90,5,4,3);
//Leg3D leg3 = Leg3D(1,1,2,90,90,90,8,7,6);
//Leg3D leg4 = Leg3D(2,2,2,90,90,90,11,10,9);

Leg3D leg1 = Leg3D(1,1,1,90,90,90,2,1,0);  // front right
Leg3D leg2 = Leg3D(2,2,1,90,90,90,5,4,3);  // front left
Leg3D leg3 = Leg3D(2,1,2,90,90,90,8,7,6);  // rear left
Leg3D leg4 = Leg3D(1,2,2,90,90,90,11,10,9);  // rear right

void setup(){
  Serial.begin(115200);
  leg1.attach();  
  leg2.attach();  
  leg3.attach();  
  leg4.attach();  


  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  pinMode(btn_C_pin, INPUT_PULLUP);

  Serial.print(Go);
  Serial.print('\t');
  Serial.println(Stay);
}

void loop(){

    dT = (millis() - tOld)*1.0e-3;
    tOld = millis();

//    Serial.print(Go);
//    Serial.print('\t');
//    Serial.print(Stay);
//    Serial.print('\t');
//    btn_B = !digitalRead(btn_B_pin);          // read if the button is being currently pressed
//    btn_B_press = btn_B && !btn_B_old;  // determine if it is a unique press
//    btn_B_old = btn_B;  // update btn_B_old according to current button state
//    
//    
//    btn_C = !digitalRead(btn_C_pin);          // read if the button is being currently pressed
//    btn_C_press = btn_C && !btn_C_old;  // determine if it is a unique press
//    btn_C_old = btn_C;  // update btn_C_old according to current button state
//    Serial.print(btn_C_press);
//    Serial.print('\t');
//
//    Tgs = Go && btn_C_press;
//    Lg = Go && !btn_C_press;
//    Tsg = Stay && btn_C_press;
//    Ls = Stay && !btn_C_press;
//    Serial.print(Go);
//    Serial.print('\t');
//    Serial.print(Stay);
//    Serial.print('\t');
//    Serial.print(Tgs);
//    Serial.print('\t');
//    Serial.print(Lg);
//    Serial.print('\t');
//    Serial.print(Tsg);
//    Serial.print('\t');
//    Serial.print(Ls);
//
//    Go = Tsg || Lg;
//    Stay = Tgs || Ls;
//    
//    if(Go) {
//        walk();
//        Serial.println("Walk");
//    }
//    else {
//        stand();
//        Serial.println("Stand");
//    }
  
    stand();
//    walk();
//    leftTurn();
//    rightTurn();
//    walkBack();


//  // Finite State Machine
//  irReading = analogRead(A4);
////  edgeDetected = irReading < 110.0;
//  edgeDetected = irReading < 5.0;
//  backTime = backTimer(Back);
////  backTime = false;
//  turnTime = turnTimer(RightTurn || LeftTurn);
////  turnTime = false;
//  bool randChoice = random(0,2)== 1;
//  
//  Lf = Fwd && !edgeDetected;
//  Tfb = Fwd && edgeDetected;
//  Lb = Back && !backTime;
//  Tbr = Back && backTime && randChoice;
//  Tbl = Back && backTime && !randChoice;
//  Lr = RightTurn && !turnTime && !edgeDetected;
//  Ll = LeftTurn && !turnTime && !edgeDetected;
//  Trb = RightTurn && edgeDetected;
//  Tlb = LeftTurn && edgeDetected;
//  Trf = RightTurn && turnTime;
//  Tlf = LeftTurn && turnTime;
//
//  Fwd = Lf || Trf || Tlf;
//  Back = Lb || Tfb || Trb || Tlb;
//  RightTurn = Tbr || Lr;
//  LeftTurn = Tbl || Ll;
//
//  if(Fwd) {
//    walk();
//  }
//  else if(Back) {
//    walkBack();
//  }
//  else if(LeftTurn) {
//    leftTurn();
//  }
//  else if(RightTurn) {
//    rightTurn();
//  }
//  else {
//    // fell through the logic!
//  }
////
//  Serial.print(irReading);
//  Serial.print('\t');
//  Serial.print(randChoice);
//  Serial.print('\t');
//  Serial.print(Fwd);
//  Serial.print('\t');
//  Serial.print(Back);
//  Serial.print('\t');
//  Serial.print(LeftTurn);
//  Serial.print('\t');
//  Serial.print(RightTurn);
  Serial.println('\t');
  
}


void stand() {
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

    leg1.update(xfr,yfr,zfr);
    leg2.update(xfl,yfl,zfl);
    leg3.update(xrl,yrl,zrl);
    leg4.update(xrr,yrr,zrr);

//    Serial.print(t1);
//    Serial.print("\t");
//    Serial.print(xfr);
//    Serial.print("\t");
//    Serial.print(yfr);
//    Serial.print("\t");
//    Serial.print(zfr);
//    Serial.print("\t");
//    Serial.print(phase);
//    Serial.println();
    
}


void walk() {
    
    float xfl = xGait(1); 
    float yfl = 0;
    float zfl = zamp*zGait(1);
    float xfr = xGait(3);
    float yfr = 0;
    float zfr = zamp*zGait(3);
    float xrl = xGait(4);
    float yrl = 0;
    float zrl = zamp*zGait(4);
    float xrr = xGait(2);
    float yrr = 0;
    float zrr = zamp*zGait(2);
    
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
//    Serial.print(millis());
//    Serial.print("\t");
//    Serial.print(xrl);
//    Serial.print("\t");
//    Serial.print(phase);
//    Serial.print(xOld[4]);
//    Serial.print("\t");
//    Serial.print(yfr);
//    Serial.print("\t");
//    Serial.print(zfr);
//    Serial.print("\t");
////    Serial.print(phase);
//    Serial.println();
}

void walkBack() {
//    float xfl = -xamp*xGait(1); 
//    float yfl = 0;
//    float zfl = zamp*zGait(1);
//    float xfr = -xamp*xGait(3);
//    float yfr = 0;
//    float zfr = zamp*zGait(3);
//    float xrl = -xamp*xGait(4);
//    float yrl = 0;
//    float zrl = zamp*zGait(4);
//    float xrr = -xamp*xGait(2);
//    float yrr = 0;
//    float zrr = zamp*zGait(2);

    float xfl = -xGait(1); 
    float yfl = 0;
    float zfl = zamp*zGait(1);
    float xfr = -xGait(3);
    float yfr = 0;
    float zfr = zamp*zGait(3);
    float xrl = -xGait(4);
    float yrl = 0;
    float zrl = zamp*zGait(4);
    float xrr = -xGait(2);
    float yrr = 0;
    float zrr = zamp*zGait(2);
    
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
//    Serial.print(xfr);
//    Serial.print("\t");
//    Serial.print(phase);
//    Serial.print(yfr);
//    Serial.print("\t");
//    Serial.print(zfr);
//    Serial.print("\t");
////    Serial.print(phase);
//    Serial.println();
}

void leftTurn() {
    float xfl = 0; 
    float yfl = yamp*yGait(1);
    float zfl = zamp*zGait(1);
    float xfr = 0;
    float yfr = -yamp*yGait(3);
    float zfr = zamp*zGait(3);
    float xrl = 0;
    float yrl = -yamp*yGait(4);
    float zrl = zamp*zGait(4);
    float xrr = 0;
    float yrr = yamp*yGait(2);
    float zrr = zamp*zGait(2);
    
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
//    Serial.print(xfr);
//    Serial.print("\t");
//    Serial.print(yfr);
//    Serial.print("\t");
//    Serial.print(zfr);
//    Serial.print("\t");
////    Serial.print(phase);
//    Serial.println();
}

void rightTurn() {
    float xfl = 0; 
    float yfl = -yamp*yGait(1);
    float zfl = zamp*zGait(1);
    float xfr = 0;
    float yfr = yamp*yGait(3);
    float zfr = zamp*zGait(3);
    float xrl = 0;
    float yrl = yamp*yGait(4);
    float zrl = zamp*zGait(4);
    float xrr = 0;
    float yrr = -yamp*yGait(2);
    float zrr = zamp*zGait(2);
    
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
//    Serial.print(xfr);
//    Serial.print("\t");
//    Serial.print(yfr);
//    Serial.print("\t");
//    Serial.print(zfr);
//    Serial.print("\t");
////    Serial.print(phase);
//    Serial.println();
}

float xGait(int n) {
    swingThres = 3*pi/2 + n*pi/2;
    swingDur = pi/2;
    phi = pi/2;


    t1 = gaitTimer()*1.0e-3;
    

    if(swingThres >= 2*pi) {
      swingThres = swingThres - 2*pi;
    }

    if((Wgait*t1 >= swingThres) && (Wgait*t1 < (swingThres + swingDur))) {
      x = xamp*cos(Wgait * 2.0 * t1 - (phi*n)*2.0);
      phase = "fast";
    }
    else if(Wgait*t1 < swingThres) {
//      x = xamp*cos(Wgait*(2.0/3.0)*t1- ((phi*n+pi))*(2.0/3.0));
//      x = xOld - vStance*dT;
      x = xOld[n-1] - vStance*dT;
      phase = "slow 1";
    }
    else {
//      x = xamp*cos(Wgait*(2.0/3.0)*t1 - (phi*n)*(2.0/3.0));
//      x = xOld - vStance*dT;
      x = xOld[n-1] - vStance*dT;
      phase = "slow 2";
    }

    if(n == 4){
      Serial.print(Wgait*t1);
      Serial.print("\t");
    }

//    xOld = x;
    xOld[n-1] = x;
    
    return x;
}

float yGait(int n) {
    // Maybe like x?
    swingThres = 3*pi/2 + n*pi/2;
    swingDur = pi/2;
    phi = pi/2;


    t1 = gaitTimer()*1.0e-3;
    

    if(swingThres >= 2*pi) {
      swingThres = swingThres - 2*pi;
    }

    if((Wgait*t1 > swingThres) && (Wgait*t1 < (swingThres + swingDur))) {
      y = cos(Wgait * 2.0 * t1 - (phi*n)*2.0);
      phase = "fast";
    }
    else if(Wgait*t1 <= swingThres) {
      y = cos(Wgait*(2.0/3.0)*t1- ((phi*n+pi))*(2.0/3.0));
      phase = "slow 1";
    }
    else {
      y = cos(Wgait*(2.0/3.0)*t1 - (phi*n)*(2.0/3.0));
      phase = "slow 2";
    }

    return y;
}

float zGait(int n) {
    t1 = t1 * 1e-3;
    swingThres =  2*pi + (n-1)*pi/2;
    swingDur = pi/2;
    phi = pi/2;


    t1 = gaitTimer()*1.0e-3;
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


// Other attempts

//    float x = 0;
//    float y = 0;
//    float z = amp*sin(freq*t);

//    float phifr = 0;
//    float phirl = 3.14159265/2;
//    float phifl = 3.14159265;
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

//    Serial.println("What would you like me to do? You can say: walk, stand, or left turn:");
//    state = Serial.read();
//
//    // send data only when you receive data:
//    if (Serial.available() > 0) {
//        if(state == "walk") {
//            // Walk
//            walk();
//            delay(5000);
//        }
//        else if(state == "stand") {
//            // Stand
//            stand();
//            delay(5000);
//        }
//        else if(state == "left turn") {
//            // Left turn
//            leftTurn();
//            delay(5000);
//        }
//        else {
//            Serial.println("Didn't catch that. You can say: walk, stand, or left turn:");
//        }
//    }


//  cycleTime = timer();
//  
//  Tlr = LeftTurn && cycleTime;
//  Ll = LeftTurn && !cycleTime;
//  Trl = RightTurn && cycleTime;
//  Lr = RightTurn && !cycleTime;
//
//  
//  LeftTurn = Ll || Trl;
//  RightTurn = Lr || Tlr;
//
//  if(LeftTurn) {
//    leftTurn();
//  }
//  else if(RightTurn) {
//    rightTurn();
//  }
//  else {
//    // fell through logic!
//  }
//
//  Serial.print(LeftTurn);
//  Serial.print('\t');
//  Serial.print(RightTurn);
//  Serial.println('\t');
