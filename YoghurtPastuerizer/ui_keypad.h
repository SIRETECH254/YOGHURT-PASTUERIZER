

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
  SCREEN_RUNNING
};

// Global Keypad Variables
extern MenuState currentMenuState;
extern ProcessState selectedManualState;
extern String inputBuffer;
extern int settingCursor; 

void initKeypadInterface();
void processKeypadDigits();
void formatTimeStr(char* buffer, unsigned long ms);

#endif
