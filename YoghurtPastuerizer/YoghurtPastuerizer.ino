



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
      
      runPIDControl(coolTemp, TARGET_HEAT_TEMP); 
      
      // Safety Override: Throttle/turn off heater if jacket gets too hot, but DO NOT fault brick the machine
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
      runCoolingControl(coolTemp, TARGET_COOL_TEMP); // Taper valve near target to prevent undershoot
      
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
    { // Scope bracket to isolate local variables and fix compile warnings
      // ACTUATORS: HEATER AND COOLER ONLY (MAINTAINING TEMPERATURE VIA TIME DUTY-CYCLE)
      digitalWrite(RELAY_AGITATOR_PIN, HIGH); // Force Agitator OFF

      static unsigned long holdingCycleStart = 0;
      static unsigned long heaterOnDuration = 0;
      static unsigned long coolerOnDuration = 0;
      static bool inContinuousMode = false;

      // Continuous-drive boundary needs a hysteresis dead zone: this check runs every
      // loop tick (~20Hz), unlike the duty-cycle tiers below which only re-evaluate once
      // per 60s window. Without the dead zone, sensor noise sitting right at the threshold
      // would rapidly flip the relay between continuous-drive and duty-cycle -- same class
      // of chatter/wear that HEAT_HYSTERESIS/COOL_HYSTERESIS guard against in control_heater.cpp.
      const float HOLD_CONTINUOUS_THRESHOLD  = 7.0;
      const float HOLD_CONTINUOUS_HYSTERESIS = 0.5; // kept under the 1C tier width so it can't skip a whole tier
      const float HOLD_REST_THRESHOLD        = 3.0;

      unsigned long now = millis();
      float error    = TARGET_HOLD_TEMP - coolTemp;
      float absError = abs(error);

      if (inContinuousMode) {
        inContinuousMode = (absError >= HOLD_CONTINUOUS_THRESHOLD - HOLD_CONTINUOUS_HYSTERESIS);
      } else {
        inContinuousMode = (absError >= HOLD_CONTINUOUS_THRESHOLD);
      }

      if (inContinuousMode) {
        // CONTINUOUS mode: deviation too large for duty cycling, run actuator flat-out
        holdingCycleStart = 0; // force duty-cycle to restart fresh once we drop out of continuous mode
        if (error > 0) {
          digitalWrite(RELAY_HEATER_PIN, LOW);   // Heater ON
          digitalWrite(RELAY_COOLING_PIN, HIGH); // Cooler OFF
          Serial.println("HOLD: >=7C under target. Continuous HEAT.");
        } else {
          digitalWrite(RELAY_HEATER_PIN, HIGH);  // Heater OFF
          digitalWrite(RELAY_COOLING_PIN, LOW);  // Cooler ON
          Serial.println("HOLD: >=7C over target. Continuous COOL.");
        }
      } else {
        // DUTY-CYCLE mode: proportional pulse within a 60s window
        if (holdingCycleStart == 0 || (now - holdingCycleStart >= 60000)) {
          holdingCycleStart = now;
          heaterOnDuration  = 0;
          coolerOnDuration  = 0;

          unsigned long pulseDuration = 0;
          if      (absError >= 6.0) pulseDuration = 20000; // 20s
          else if (absError >= 5.0) pulseDuration = 15000; // 15s
          else if (absError >= 4.0) pulseDuration = 10000; // 10s
          else if (absError >= HOLD_REST_THRESHOLD) pulseDuration = 5000; // 5s
          // < 3C -> rest

          if (pulseDuration == 0) {
            Serial.println("HOLD: Within 3C. Rest cycle.");
          } else if (error > 0) {
            heaterOnDuration = pulseDuration;
            Serial.print("HOLD: Under target. Pulsing HEATER for ");
            Serial.print(pulseDuration / 1000);
            Serial.println("s.");
          } else {
            coolerOnDuration = pulseDuration;
            Serial.print("HOLD: Over target. Pulsing COOLING for ");
            Serial.print(pulseDuration / 1000);
            Serial.println("s.");
          }
        }

        unsigned long cycleProgress = now - holdingCycleStart;
        if (heaterOnDuration > 0 && cycleProgress < heaterOnDuration) {
          digitalWrite(RELAY_HEATER_PIN, LOW);   // Heater ON
          digitalWrite(RELAY_COOLING_PIN, HIGH); // Cooler OFF
        } else if (coolerOnDuration > 0 && cycleProgress < coolerOnDuration) {
          digitalWrite(RELAY_HEATER_PIN, HIGH);  // Heater OFF
          digitalWrite(RELAY_COOLING_PIN, LOW);  // Cooler ON
        } else {
          digitalWrite(RELAY_HEATER_PIN, HIGH);  // Heater OFF
          digitalWrite(RELAY_COOLING_PIN, HIGH); // Cooler OFF
        }
      }
      
      // Countdown time handling
      if (!phaseTimerActive) {
        // Once temperature is within +/- 2.0C of target, initiate holding time clock
        if (coolTemp >= TARGET_HOLD_TEMP - 2.0 && coolTemp <= TARGET_HOLD_TEMP + 2.0) {
           phaseTimerActive = true;
           phaseStartTime = millis();
           holdingCycleStart = millis(); // Reset cycle baseline
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
