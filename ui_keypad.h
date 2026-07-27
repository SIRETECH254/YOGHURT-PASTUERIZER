

#ifndef UI_KEYPAD_H
#define UI_KEYPAD_H

#include <Arduino.h> 
#include "states_state_machine.h" 

// UI Screen Enumerator Map
enum MenuState {
  SCREEN_HOME,
  SCREEN_AUTO_READY,
  SCREEN_MANUAL_MENU,
  SCREEN_MANUAL_READY,
  SCREEN_SETTINGS_MAIN,
  SCREEN_SET_HEAT,
  SCREEN_SET_COOL,
  SCREEN_SET_MIX,
  SCREEN_SET_HOLD,
  SCREEN_RUNNING,
  SCREEN_RESUME_PROMPT   // Operator confirmation before resuming a run interrupted by power loss
};

// Global Keypad Variables
extern MenuState currentMenuState;
extern ProcessState selectedManualState;
extern String inputBuffer;
extern int settingCursor; 

// Pending resume-after-power-loss data. Populated by setup() (from
// memory_storage's loadRunState()) when an interrupted run is found.
// Read by processKeypadDigits() when the operator confirms/declines
// on SCREEN_RESUME_PROMPT.
extern ProcessState pendingResumeState;
extern OperationMode pendingResumeMode;
extern ProcessState pendingResumeManual;
extern bool pendingResumeTimerActive;
extern unsigned long pendingResumeElapsedMs;
extern unsigned long pendingResumeOutageMs;

void initKeypadInterface();
void processKeypadDigits();
void formatTimeStr(char* buffer, unsigned long ms);

#endif
