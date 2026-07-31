



#include <Arduino.h>
#include "config_settings.h"
#include "sensors_temperature.h"
#include "control_heater.h"
#include "display_lcd.h"
#include "states_state_machine.h"
#include "ui_keypad.h"
#include "memory_storage.h"

void setup() {
  Serial.begin(9600);
  initHeater();
  initTemperatureSensor();
  runSensorDiagnostics(); 
  initLCD();
  initKeypadInterface();  
  initMemoryStorage();

  loadSettings(); // Pull in any previously saved temps/durations, overriding the compiled defaults

  // Auto-resume any run interrupted by a power loss.
  ProcessState resumeState; OperationMode resumeMode; ProcessState resumeManual;
  bool resumeTimerActive; unsigned long resumeElapsedMs; unsigned long resumeOutageMs;
  if (loadRunState(resumeState, resumeMode, resumeManual, resumeTimerActive, resumeElapsedMs, resumeOutageMs)) {
    Serial.print("MEMORY: Auto-resuming interrupted run. Outage was approx ");
    Serial.print(resumeOutageMs / 1000);
    Serial.println("s.");
    currentMode             = resumeMode;
    selectedManualState     = resumeManual;
    currentState            = resumeState;
    currentMenuState        = SCREEN_RUNNING;
    if (resumeTimerActive) {
      phaseTimerActive = true;
      phaseStartTime   = millis() - resumeElapsedMs;
    } else {
      phaseTimerActive = false;
    }
  }
}

void transitionToNextState(ProcessState nextAutoState) {
  turnOffAllActuators(); // Ensures everything is dead before the next phase turns anything on
  resetPhaseTimer();
  if (currentMode == MODE_AUTO) {
    currentState = nextAutoState;
  } else {
    currentState = COMPLETE;
  }

  if (currentState == COMPLETE) {
    clearRunState(); // Finished normally -- nothing to resume on next boot
  } else {
    saveRunState(currentState, currentMode, selectedManualState, false, 0); // Fresh phase, timer not yet active
  }
}

void loop() {
  float heatTemp = readHeatingTemperature();
  float coolTemp = readCoolingTemperature();

  // Once-per-second serial telemetry so you can watch jacket-vs-product on the monitor
  static unsigned long lastTempLog = 0;
  if (millis() - lastTempLog >= 1000) {
    lastTempLog = millis();
    Serial.print("JACKET [A1]: ");
    Serial.print(heatTemp, 1);
    Serial.print(" C  |  PRODUCT [A0]: ");
    Serial.print(coolTemp, 1);
    Serial.println(" C");
  }

  // Track consecutive sensor faults to filter out single-frame EMI relay noise spikes
  static int consecutiveFaults = 0;
  if (heatTemp >= 999.0 || coolTemp >= 999.0) {
    consecutiveFaults++;
  } else {
    consecutiveFaults = 0; // Reset counter on valid reading
  }

  // Only trigger emergency stop if the sensor fault is solid and continuous (10 loops = approx 1 second)
  if (consecutiveFaults >= 10) {
    Serial.println("CRITICAL: Sustained sensor disconnection detected! Tripping emergency safety...");
    currentState = FAULT_ERROR;
  }
  
  // 1. DEBUNCED PHYSICAL BUTTON CHECK
  if (startButtonPressed()) {
    if (currentMenuState == SCREEN_AUTO_READY) {
      currentState = HEATING;
      currentMenuState = SCREEN_RUNNING;
    } 
    else if (currentMenuState == SCREEN_MANUAL_READY) {
      currentState = selectedManualState;
      currentMenuState = SCREEN_RUNNING;
    }
  }

  if (stopButtonPressed()) {
    Serial.println("MANUAL STOP BUTTON CONFIRMED");
    currentState = FAULT_ERROR;
  }

  // 2. MATRIX KEYPAD PROCESS 
  processKeypadDigits();

  // 3. CORE STATE MACHINE ENGINE
  switch(currentState) {
    case SYSTEM_IDLE:
      turnOffAllActuators();
      resetPhaseTimer();
      consecutiveFaults = 0;
      break;
        
    case HEATING:
      // ACTUATORS: HEATER AND AGITATOR
      turnOnAgitator();                       // Agitator ON during heating to ensure uniform mixture
      digitalWrite(RELAY_COOLING_PIN, HIGH);  // Force Cooling Valve OFF
      
      // Jacket-driven maintenance: heater cycles to hold jacket at TARGET_HEAT_TEMP.
      // Product asymptotes to that temp with zero overshoot (heat only flows hot -> cold).
      runPIDControl(heatTemp, TARGET_HEAT_TEMP);

      // Safety backstop: cut heater if jacket exceeds the absolute safety ceiling.
      if (heatTemp >= (MAX_SAFE_TEMP - 5.0)) {
        turnOffHeater();
      }

      if (!phaseTimerActive) {
        if (coolTemp >= TARGET_HEAT_TEMP) {
          phaseTimerActive = true;
          phaseStartTime = millis();
        }
      } else {
        if (millis() - phaseStartTime >= HEAT_DUR_MS) {
          transitionToNextState(COOLING);
        }
      }
      break;

    case COOLING:
      // ACTUATOR: COOLING VALVE ONLY
      turnOffHeater();                        // Force Heater OFF
      digitalWrite(RELAY_AGITATOR_PIN, HIGH); // Force Agitator OFF
      // Jacket-driven maintenance: valve cycles to hold jacket at TARGET_COOL_TEMP.
      // Product asymptotes down to that temp with zero undershoot.
      runCoolingControl(heatTemp, TARGET_COOL_TEMP);
      
      if (!phaseTimerActive) {
        if (coolTemp <= TARGET_COOL_TEMP) {
          phaseTimerActive = true;
          phaseStartTime = millis();
        }
      } else {
        if (millis() - phaseStartTime >= COOL_DUR_MS) {
          transitionToNextState(MIXING);
        }
      }
      break;
      
    case MIXING:
      // ACTUATOR: AGITATOR ONLY
      turnOffHeater();                        // Force Heater OFF
      digitalWrite(RELAY_COOLING_PIN, HIGH);  // Force Cooling Valve OFF
      turnOnAgitator();                       // Turn Agitator ON
      
      if (!phaseTimerActive) {
        phaseTimerActive = true;
        phaseStartTime = millis();
      } else {
        if (millis() - phaseStartTime >= MIX_DUR_MS) {
          transitionToNextState(HOLDING);
        }
      }
      break;
        
    case HOLDING:
    { // Scope bracket -- required so the local declarations below don't "jump" past following case labels
      // ACTUATORS: HEATER AND COOLER (jacket-driven bidirectional maintenance)
      digitalWrite(RELAY_AGITATOR_PIN, HIGH); // Force Agitator OFF

      // Rest zone with hysteresis: enter rest at +/- 3C, only exit at +/- 5C.
      // The 2C dead band between entry and exit is what stops relay flicker at the
      // boundary -- without it, probe noise (~0.5C EMI wobble) would flip the state
      // on every loop iteration. Small drifts self-correct via ambient equilibrium.
      static bool inRestZone = true;
      float holdDev = fabs(heatTemp - TARGET_HOLD_TEMP);
      if (inRestZone) {
        if (holdDev > 5.0) inRestZone = false; // Drifted far enough to need active correction
      } else {
        if (holdDev <= 3.0) inRestZone = true; // Back inside tolerance
      }

      if (inRestZone) {
        turnOffHeater();
        turnOffCoolingValve();
      } else if (heatTemp < TARGET_HOLD_TEMP) {
        turnOffCoolingValve();
        runPIDControl(heatTemp, TARGET_HOLD_TEMP);
      } else {
        turnOffHeater();
        runCoolingControl(heatTemp, TARGET_HOLD_TEMP);
      }

      // Countdown time handling
      if (!phaseTimerActive) {
        // Once product temperature is within +/- 2.0C of target, initiate holding time clock
        if (coolTemp >= TARGET_HOLD_TEMP - 2.0 && coolTemp <= TARGET_HOLD_TEMP + 2.0) {
           phaseTimerActive = true;
           phaseStartTime = millis();
        }
      } else {
        if (millis() - phaseStartTime >= HOLD_DUR_MS) {
          transitionToNextState(COMPLETE);
        }
      }
      break;
    } // Close scope bracket
        
    case COMPLETE:
      turnOffAllActuators();
      soundBuzzer();
      break;
        
    case FAULT_ERROR:
      emergencyShutdown();
      if (stopButtonPressed()) {
         currentState = SYSTEM_IDLE;
         currentMode = MODE_NONE;
         currentMenuState = SCREEN_HOME; 
         turnOffAllActuators(); 
         clearRunState(); // Operator acknowledged the stop -- nothing to resume
      }
      break;
  }

  // Periodically checkpoint run state to memory so we can resume correctly
  // after a power loss. Throttled to once every 30s to protect EEPROM
  // write endurance -- do not save this on every loop iteration.
  static unsigned long lastCheckpoint = 0;
  if (currentState == HEATING || currentState == COOLING || currentState == MIXING || currentState == HOLDING) {
    if (millis() - lastCheckpoint >= 30000) {
      lastCheckpoint = millis();
      unsigned long elapsed = phaseTimerActive ? (millis() - phaseStartTime) : 0;
      saveRunState(currentState, currentMode, selectedManualState, phaseTimerActive, elapsed);
    }
  }

  updateDisplay(coolTemp, (int)currentState);
  delay(50);
}
