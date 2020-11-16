// Initialize transitions
bool Twt;
bool Lwait;
bool Ttw;
bool Lt;

// Initialize states
bool TMR_waiting = true;
bool TMR_timing = false;

// Other values
float TMR_elapsed = 0;  // elapsed time (ms)
unsigned long TMR_starttime = millis;  // time at which timer started (ms)
const int TMR_duration = 1000;   // duration of timer (ms)

bool timeComplete = false;
bool TMR_enable = true;

float timer() {

  // BLOCK 2: State Transition Logic
  //---------------------------------------- 
  Twt = TMR_waiting && TMR_enable;  // transition from waiting to timing
  Lwait = TMR_waiting && !TMR_enable;  // latch on waiting
  Ttw = TMR_timing && !TMR_enable;  // transition from timing to waiting
  Lt = TMR_timing && TMR_enable;    // latch on timing
  
    
  // BLOCK 3: Update States
  //---------------------------------------- 
  TMR_waiting = Lwait || Ttw;
  TMR_timing = Twt || Lt;
  
  
  // BLOCK 4: Set Outputs and Old Variables
  //----------------------------------------   

  if(TMR_timing && TMR_elapsed < TMR_duration){
    // If the timer is still running, update elapsed time
    TMR_elapsed = millis() - TMR_starttime;
    timeComplete = false;
  }
  else if(TMR_timing && TMR_elapsed >= TMR_duration){
    // If the timer has exceeded the allowed duration, stop the timer
    TMR_elapsed = 0; //TMR_duration;
    timeComplete = true;
    TMR_enable = false;
  }
  else if(TMR_waiting) {
    // If waiting, do not run the timer
    TMR_starttime = millis();
    TMR_elapsed = 0;
    timeComplete = false;
    TMR_enable = true;
  }

//  return timeComplete;  // return whether the timer has reached duration or not
  return TMR_elapsed;
}
