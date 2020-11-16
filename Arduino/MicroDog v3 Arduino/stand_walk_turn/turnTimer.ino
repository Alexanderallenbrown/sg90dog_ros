// Initialize transitions
bool Twt2;
bool Lwait2;
bool Ttw2;
bool Lt2;

// Initialize states
bool TMR_waiting2 = true;
bool TMR_timing2 = false;

// Other values
unsigned long TMR_elapsed2 = 0;  // elapsed time (ms)
unsigned long TMR_starttime2 = millis;  // time at which timer started (ms)
const int TMR_duration2 = 2000;   // duration of timer (ms)
bool timeComplete2 = false;

bool turnTimer(bool TMR_enable2) {

  // BLOCK 2: State Transition Logic
  //---------------------------------------- 
  Twt2 = TMR_waiting2 && TMR_enable2;  // transition from waiting to timing
  Lwait2 = TMR_waiting2 && !TMR_enable2;  // latch on waiting
  Ttw2 = TMR_timing2 && !TMR_enable2;  // transition from timing to waiting
  Lt2 = TMR_timing2 && TMR_enable2;    // latch on timing
  
    
  // BLOCK 3: Update States
  //---------------------------------------- 
  TMR_waiting2 = Lwait2 || Ttw2;
  TMR_timing2 = Twt2 || Lt2;
  
  
  // BLOCK 4: Set Outputs and Old Variables
  //----------------------------------------   

  if(TMR_timing2 && TMR_elapsed2 < TMR_duration2){
    // If the timer is still running, update elapsed time
    TMR_elapsed2 = millis() - TMR_starttime2;
    timeComplete2 = false;
  }
  else if(TMR_timing2 && TMR_elapsed2 >= TMR_duration2){
    // If the timer has exceeded the allowed duration, stop the timer
    TMR_elapsed2 = TMR_duration2;
    timeComplete2 = true;
  }
  else if(TMR_waiting2) {
    // If waiting, do not run the timer
    TMR_starttime2 = millis();
    TMR_elapsed2 = 0;
    timeComplete2 = false;
  }

  return timeComplete2;  // return whether the timer has reached duration or not
}
