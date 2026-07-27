



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

  // Check for a run that was interrupted by a power loss. Don't resume
  // automatically -- stage the data and let the operator confirm on the
  // LCD, since an unattended multi-hour outage may mean the product can no
  // longer be trusted to just pick back up where it left off.
  if (loadRunState(pendingResumeState, pendingResumeMode, pendingResumeManual,
                    pendingResumeTimerActive, pendingResumeElapsedMs, pendingResumeOutageMs)) {
    Serial.print("MEMORY: Found interrupted run. Outage was approx ");
    Serial.print(pendingResumeOutageMs / 1000);
    Serial.println("s. Awaiting operator confirmation.");
    currentMenuState = SCREEN_RESUME_PROMPT;
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
      // ACTUATOR: COOLING VALVE ONLY
      turnOffHeater();                        // Force Heater OFF
      digitalWrite(RELAY_AGITATOR_PIN, HIGH); // Force Agitator OFF
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
      
      unsigned long now = millis();

      // Check if we need to start a new 1-minute duty control cycle
      if (holdingCycleStart == 0 || (now - holdingCycleStart >= 60000)) {
        holdingCycleStart = now;
        
        float error = TARGET_HOLD_TEMP - coolTemp;
        
        // Reset durations
        heaterOnDuration = 0;
        coolerOnDuration = 0;

        // Stepped correction table: below HOLD_TIER1_C is the rest/deadband
        // zone, then each tier fires a fixed pulse length. No scaling between
        // tiers -- just the flat value for whichever band the deviation falls in.
        const float HOLD_TIER1_C = 2.5;                 // >= this deviation: short nudge
        const float HOLD_TIER2_C = 4.0;                 // >= this deviation: medium pulse
        const float HOLD_TIER3_C = 6.0;                 // >= this deviation: max pulse
        const unsigned long HOLD_TIER1_MS = 5000;        // 5s
        const unsigned long HOLD_TIER2_MS = 15000;       // 15s
        const unsigned long HOLD_TIER3_MS = 25000;       // 25s

        float absError = abs(error);
        unsigned long pulseDuration = 0; // stays 0 -> rest if below HOLD_TIER1_C

        if (absError >= HOLD_TIER3_C) {
          pulseDuration = HOLD_TIER3_MS;
        } else if (absError >= HOLD_TIER2_C) {
          pulseDuration = HOLD_TIER2_MS;
        } else if (absError >= HOLD_TIER1_C) {
          pulseDuration = HOLD_TIER1_MS;
        }

        if (pulseDuration == 0) {
          Serial.println("HOLD: Temperature stable. Rest cycle.");
        }
        else if (error > 0) {
          // Milk is cold: pulse HEATER for this tier's fixed duration
          heaterOnDuration = pulseDuration;
          Serial.print("HOLD: Under target. Pulsing HEATER for ");
          Serial.print(pulseDuration / 1000);
          Serial.println("s.");
        } else {
          // Milk is hot: pulse COOLING valve for this tier's fixed duration
          coolerOnDuration = pulseDuration;
          Serial.print("HOLD: Over target. Pulsing COOLING for ");
          Serial.print(pulseDuration / 1000);
          Serial.println("s.");
        }
      }

      // Execute calculated duty cycles within the active 1-minute window
      unsigned long cycleProgress = now - holdingCycleStart;

      if (heaterOnDuration > 0 && cycleProgress < heaterOnDuration) {
        // Run Heater Burst
        digitalWrite(RELAY_HEATER_PIN, LOW);     // Heater ON
        digitalWrite(RELAY_COOLING_PIN, HIGH);   // Cooler OFF
      } 
      else if (coolerOnDuration > 0 && cycleProgress < coolerOnDuration) {
        // Run Cooler Burst
        digitalWrite(RELAY_HEATER_PIN, HIGH);    // Heater OFF
        digitalWrite(RELAY_COOLING_PIN, LOW);     // Cooler ON
      } 
      else {
        // Rest state for the remainder of the 1-minute cycle (resulting in 25s of rest if duty was 35s)
        digitalWrite(RELAY_HEATER_PIN, HIGH);    // Heater OFF
        digitalWrite(RELAY_COOLING_PIN, HIGH);   // Cooler OFF
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
