



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
  turnOffAllActuators(true); // Ensures everything is dead before the next phase turns anything on
  resetActuatorControl();
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

  // Emergency safety check: if sensors return 999.0 (sustained disconnection confirmed)
  if (heatTemp >= 999.0 || coolTemp >= 999.0) {
    Serial.println("CRITICAL: Sustained sensor disconnection detected! Tripping emergency safety...");
    currentState = FAULT_ERROR;
  }
  
  // 1. DEBUNCED PHYSICAL BUTTON CHECK
  if (startButtonPressed()) {
    resetActuatorControl();
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
      turnOffAllActuators(false);
      resetPhaseTimer();
      break;
        
    case HEATING:
      // ACTUATORS: HEATER AND AGITATOR
      turnOnAgitator();                       // Agitator ON during heating to ensure uniform mixture
      setCoolingRelay(false);                 // Force Cooling Valve OFF
      
      runPIDControl(heatTemp, TARGET_HEAT_TEMP);
      
      // Safety Override: Throttle heater if jacket approaches max safe limit (105°C)
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
      // ACTUATORS: COOLING VALVE AND AGITATOR
      turnOffHeater();                        // Force Heater OFF
      turnOnAgitator();                       // Agitator ON during cooling to ensure uniform mixture
      runCoolingControl(coolTemp, TARGET_COOL_TEMP); // Product-driven: cuts off at target + 5C to coast smoothly down
      
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
      setCoolingRelay(false);                 // Force Cooling Valve OFF
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
      // ACTUATORS: HEATER CONTROLLED OFF PRODUCT CORE WITH JACKET SAFETY CAP
      setAgitatorRelay(false);                // Force Agitator OFF (undisturbed incubation)
      runHoldingControl(coolTemp, heatTemp, TARGET_HOLD_TEMP);

      // Countdown time handling - PRODUCT DRIVEN
      if (!phaseTimerActive) {
        // Once PRODUCT temperature reaches within +/- 1.0C of target, initiate holding time clock
        if (coolTemp >= TARGET_HOLD_TEMP - 1.0 && coolTemp <= TARGET_HOLD_TEMP + 1.0) {
           phaseTimerActive = true;
           phaseStartTime = millis();
        }
      } else {
        if (millis() - phaseStartTime >= HOLD_DUR_MS) {
          transitionToNextState(COMPLETE);
        }
      }
      break;

    case COMPLETE:
      turnOffAllActuators(false);
      soundBuzzer();
      break;
        
    case FAULT_ERROR:
      emergencyShutdown();
      if (stopButtonPressed()) {
         currentState = SYSTEM_IDLE;
         currentMode = MODE_NONE;
         currentMenuState = SCREEN_HOME; 
         turnOffAllActuators(true); 
         resetActuatorControl();
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

  // Periodic Serial heartbeat -- HEATING/COOLING/MIXING/HOLDING don't otherwise
  // print anything during normal operation, so without this the monitor shows
  // only the one-time setup() diagnostics and then goes silent.
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();

    // Relays are active-low; digitalRead() on an OUTPUT pin reads back what
    // we last wrote, so this reflects the actual actuator state right now.
    bool heaterOn   = (digitalRead(RELAY_HEATER_PIN)   == LOW);
    bool coolerOn   = (digitalRead(RELAY_COOLING_PIN)  == LOW);
    bool agitatorOn = (digitalRead(RELAY_AGITATOR_PIN) == LOW);

    Serial.print("[state=");
    Serial.print(currentState);
    Serial.print("] jacket=");
    Serial.print(heatTemp, 1);
    Serial.print("C product=");
    Serial.print(coolTemp, 1);
    Serial.print("C | HEATER=");
    Serial.print(heaterOn ? "ON" : "off");
    Serial.print(" COOLER=");
    Serial.print(coolerOn ? "ON" : "off");
    Serial.print(" AGITATOR=");
    Serial.println(agitatorOn ? "ON" : "off");
  }

  updateDisplay(coolTemp, (int)currentState);
  delay(50);
}
