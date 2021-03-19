// Initialize transitions
bool Twt1;
bool Lwait1;
bool Ttw1;
bool Lt1;

// Initialize states
bool TMR_waiting1 = true;
bool TMR_timing1 = false;

// Other values
unsigned long TMR_elapsed1 = 0;  // elapsed time (ms)
unsigned long TMR_starttime1 = millis;  // time at which timer started (ms)
const int TMR_duration1 = 500;   // duration of timer (ms)
bool timeComplete1 = false;

bool delayReadingTimer(bool TMR_enable1) {

  // BLOCK 2: State Transition Logic
  //---------------------------------------- 
  Twt1 = TMR_waiting1 && TMR_enable1;  // transition from waiting to timing
  Lwait1 = TMR_waiting1 && !TMR_enable1;  // latch on waiting
  Ttw1 = TMR_timing1 && !TMR_enable1;  // transition from timing to waiting
  Lt1 = TMR_timing1 && TMR_enable1;    // latch on timing
  
    
  // BLOCK 3: Update States
  //---------------------------------------- 
  TMR_waiting1 = Lwait1 || Ttw1;
  TMR_timing1 = Twt1 || Lt1;
  
  
  // BLOCK 4: Set Outputs and Old Variables
  //----------------------------------------   

  if(TMR_timing1 && TMR_elapsed1 < TMR_duration1){
    // If the timer is still running, update elapsed time
    TMR_elapsed1 = millis() - TMR_starttime1;
    timeComplete1 = false;
  }
  else if(TMR_timing1 && TMR_elapsed1 >= TMR_duration1){
    // If the timer has exceeded the allowed duration, stop the timer
    TMR_elapsed1 = TMR_duration1;
    timeComplete1 = true;
  }
  else if(TMR_waiting1) {
    // If waiting, do not run the timer
    TMR_starttime1 = millis();
    TMR_elapsed1 = 0;
    timeComplete1 = false;
  }

  return timeComplete1;  // return whether the timer has reached duration or not
}
