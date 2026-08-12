


#include <Arduino.h>
#include "states_state_machine.h"
#include "config_settings.h"
#include "control_heater.h"

// Instantiate global variables
ProcessState currentState = SYSTEM_IDLE;
OperationMode currentMode = MODE_NONE;

bool phaseTimerActive = false;
unsigned long phaseStartTime = 0;

// Industrial-grade Software Debouncing for Start Button
bool startButtonPressed() {
  static unsigned long lastLowTime = 0;
  static bool lastState = HIGH;
  bool currentPinState = digitalRead(BUTTON_START_PIN);
  
  if (currentPinState == LOW) {
    if (lastState == HIGH) {
      lastLowTime = millis(); // Button transitioned to pressed
    }
    lastState = LOW;
    // Must be held LOW consistently for at least 100ms to filter out transient noise
    if (millis() - lastLowTime >= 100) {
      return true;
    }
  } else {
    lastState = HIGH;
  }
  return false;
}

// Industrial-grade Software Debouncing for Stop Button
bool stopButtonPressed() {
  static unsigned long lastLowTime = 0;
  static bool lastState = HIGH;
  bool currentPinState = digitalRead(BUTTON_STOP_PIN);
  
  if (currentPinState == LOW) {
    if (lastState == HIGH) {
      lastLowTime = millis(); // Button transitioned to pressed
    }
    lastState = LOW;
    // Must be held LOW consistently for at least 100ms to filter out transient noise
    if (millis() - lastLowTime >= 100) {
      return true;
    }
  } else {
    lastState = HIGH;
  }
  return false;
}

void soundBuzzer() {
  static unsigned long lastBuzzerToggle = 0;
  static bool buzzerState = false;
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastBuzzerToggle >= 500) {
    lastBuzzerToggle = currentMillis;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
}

void emergencyShutdown() {
  turnOffAllActuators();
  digitalWrite(BUZZER_PIN, HIGH); // Alarm active continuous
  Serial.println("EMERGENCY SHUTDOWN ACTIVATED");
}

void resetPhaseTimer() {
  phaseTimerActive = false;
  phaseStartTime = 0;
}
