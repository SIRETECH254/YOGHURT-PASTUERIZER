


#include <Arduino.h>
#include <Keypad.h>
#include "ui_keypad.h"
#include "config_settings.h"
#include "states_state_machine.h"
#include "memory_storage.h"

// Hardware Matrix Key Mapping Definition
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[KEYPAD_ROWS] = {22, 23, 24, 25}; 
byte colPins[KEYPAD_COLS] = {26, 27, 28}; 

Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

MenuState currentMenuState = SCREEN_HOME;
ProcessState selectedManualState = SYSTEM_IDLE;
String inputBuffer = "";
int settingCursor = 0; // 0 = Temp, 1 = Time Duration

// Pending resume-after-power-loss data, staged by setup() before the
// operator has confirmed anything on SCREEN_RESUME_PROMPT
ProcessState pendingResumeState = SYSTEM_IDLE;
OperationMode pendingResumeMode = MODE_NONE;
ProcessState pendingResumeManual = SYSTEM_IDLE;
bool pendingResumeTimerActive = false;
unsigned long pendingResumeElapsedMs = 0;
unsigned long pendingResumeOutageMs = 0;

// Default Factory Settings
float TARGET_HEAT_TEMP = 85.0;
unsigned long HEAT_DUR_MS = 0; // 0 Minutes

float TARGET_COOL_TEMP = 45.0;
unsigned long COOL_DUR_MS = 0; // 0 Minutes

unsigned long MIX_DUR_MS = 120000;  // 2 Minutes

float TARGET_HOLD_TEMP = 45.0; 
unsigned long HOLD_DUR_MS = 28800000; // 8 Hours

void initKeypadInterface() {}

void formatTimeStr(char* buffer, unsigned long ms) {
  unsigned long totSec = ms / 1000;
  int h = totSec / 3600;
  int m = (totSec % 3600) / 60;
  int s = totSec % 60;
  sprintf(buffer, "%02d:%02d:%02d", h, m, s);
}

// Converts HHMMSS inputs to operational milliseconds
unsigned long parseTimeInput(String in) {
  while(in.length() < 6) in = "0" + in; // Pad left string
  long h = in.substring(0,2).toInt();
  long m = in.substring(2,4).toInt();
  long s = in.substring(4,6).toInt();
  return (h * 3600000) + (m * 60000) + (s * 1000);
}

void processKeypadDigits() {
  char key = customKeypad.getKey();
  if (!key) return;

  // Global exit/back key logic map
  if (key == '*') {
    inputBuffer = "";
    if (currentMenuState == SCREEN_RESUME_PROMPT) {
      // Operator declined -- discard the saved run and boot fresh
      clearRunState();
      currentMenuState = SCREEN_HOME;
      return;
    }
    if (currentMenuState == SCREEN_SETTINGS_MAIN || currentMenuState == SCREEN_AUTO_READY || currentMenuState == SCREEN_MANUAL_MENU) {
      currentMenuState = SCREEN_HOME;
    } else if (currentMenuState == SCREEN_MANUAL_READY) {
      currentMenuState = SCREEN_MANUAL_MENU;
    } else if (currentMenuState == SCREEN_SET_HEAT || currentMenuState == SCREEN_SET_COOL || currentMenuState == SCREEN_SET_MIX || currentMenuState == SCREEN_SET_HOLD) {
      currentMenuState = SCREEN_SETTINGS_MAIN;
    }
    return;
  }

  switch (currentMenuState) {
    case SCREEN_HOME:
      if (key == '1') { currentMode = MODE_AUTO; currentMenuState = SCREEN_AUTO_READY; }
      if (key == '2') { currentMode = MODE_MANUAL; currentMenuState = SCREEN_MANUAL_MENU; }
      if (key == '3') { currentMenuState = SCREEN_SETTINGS_MAIN; }
      break;

    case SCREEN_AUTO_READY:
      // Start button is strictly physical
      break;

    case SCREEN_MANUAL_MENU:
      if (key == '1') { selectedManualState = HEATING; currentMenuState = SCREEN_MANUAL_READY; }
      if (key == '2') { selectedManualState = COOLING; currentMenuState = SCREEN_MANUAL_READY; }
      if (key == '3') { selectedManualState = MIXING;  currentMenuState = SCREEN_MANUAL_READY; }
      if (key == '4') { selectedManualState = HOLDING; currentMenuState = SCREEN_MANUAL_READY; }
      break;
      
    case SCREEN_MANUAL_READY:
      // Start button is strictly physical
      break;

    case SCREEN_SETTINGS_MAIN:
      if (key == '1') { currentMenuState = SCREEN_SET_HEAT; settingCursor = 0; }
      if (key == '2') { currentMenuState = SCREEN_SET_COOL; settingCursor = 0; }
      if (key == '3') { currentMenuState = SCREEN_SET_MIX; settingCursor = 1; } // Mix has no temp parameter
      if (key == '4') { currentMenuState = SCREEN_SET_HOLD; settingCursor = 0; }
      break;

    case SCREEN_SET_HEAT:
    case SCREEN_SET_COOL:
    case SCREEN_SET_MIX:
    case SCREEN_SET_HOLD:
      if (key >= '0' && key <= '9') {
        if (inputBuffer.length() < 6) inputBuffer += key;
      } 
      else if (key == '#') { 
        long val = inputBuffer.toInt();
        inputBuffer = ""; 
        
        if (currentMenuState == SCREEN_SET_HEAT) {
          if (settingCursor == 0) { TARGET_HEAT_TEMP = val; settingCursor = 1; }
          else { HEAT_DUR_MS = parseTimeInput(String(val)); currentMenuState = SCREEN_SETTINGS_MAIN; saveSettings(); }
        }
        else if (currentMenuState == SCREEN_SET_COOL) {
          if (settingCursor == 0) { TARGET_COOL_TEMP = val; settingCursor = 1; }
          else { COOL_DUR_MS = parseTimeInput(String(val)); currentMenuState = SCREEN_SETTINGS_MAIN; saveSettings(); }
        }
        else if (currentMenuState == SCREEN_SET_MIX) {
          MIX_DUR_MS = parseTimeInput(String(val)); 
          currentMenuState = SCREEN_SETTINGS_MAIN; 
          saveSettings();
        }
        else if (currentMenuState == SCREEN_SET_HOLD) {
          if (settingCursor == 0) { TARGET_HOLD_TEMP = val; settingCursor = 1; }
          else { HOLD_DUR_MS = parseTimeInput(String(val)); currentMenuState = SCREEN_SETTINGS_MAIN; saveSettings(); }
        }
      }
      break;
      
    case SCREEN_RUNNING:
      if (currentState == COMPLETE) {
         if (key == '*') {
           currentMenuState = (currentMode == MODE_MANUAL) ? SCREEN_MANUAL_MENU : SCREEN_HOME;
         }
      }
      break;

    case SCREEN_RESUME_PROMPT:
      if (key == '#') {
        // Operator confirmed: apply the staged resume data
        currentMode = pendingResumeMode;
        selectedManualState = pendingResumeManual;
        currentState = pendingResumeState;
        currentMenuState = SCREEN_RUNNING;

        if (pendingResumeTimerActive) {
          // The countdown had already started before power loss -- restore
          // it relative to the new millis() clock so remaining time is correct.
          phaseTimerActive = true;
          phaseStartTime = millis() - pendingResumeElapsedMs;
        } else {
          // Countdown hadn't started yet -- let the normal "reached temp"
          // check in loop() re-arm it naturally.
          phaseTimerActive = false;
        }
      }
      // '*' (decline) is handled by the global exit/back key logic above
      break;
  }
}
