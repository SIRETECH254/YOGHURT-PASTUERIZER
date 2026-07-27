


#ifndef STATES_STATE_MACHINE_H
#define STATES_STATE_MACHINE_H

// Finite State Machine Process Configurations
enum ProcessState {
  SYSTEM_IDLE,
  HEATING,
  COOLING,
  MIXING,
  HOLDING,
  COMPLETE,
  FAULT_ERROR
};

// Global Operation Routing Modes
enum OperationMode {
  MODE_NONE,
  MODE_AUTO,
  MODE_MANUAL
};

// Global Tracking State Variables
extern ProcessState currentState;
extern OperationMode currentMode;

// Timing Control Signals
extern bool phaseTimerActive;
extern unsigned long phaseStartTime;

// Core Hardware Buttons
bool startButtonPressed();
bool stopButtonPressed(); 

void soundBuzzer();
void emergencyShutdown();
void resetPhaseTimer();

#endif
