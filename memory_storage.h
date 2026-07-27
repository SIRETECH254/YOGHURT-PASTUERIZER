

#ifndef MEMORY_STORAGE_H
#define MEMORY_STORAGE_H

#include "states_state_machine.h"

// Call once from setup(), after Serial.begin(). Starts I2C and the RTC.
void initMemoryStorage();

// True if the RTC's own backup battery ever died (its clock value can't be
// trusted). This is independent of whether the Arduino itself lost power --
// the whole point of the RTC is that it keeps ticking even when the Arduino
// doesn't.
bool rtcBatteryLost();

// --- Persisted user settings (temps + durations) --------------------------
// Loads saved settings from internal EEPROM into the live TARGET_*/*_DUR_MS
// globals. Returns true if valid saved settings were found and applied,
// false if EEPROM was empty/uninitialized (globals keep their compiled-in
// defaults from ui_keypad.cpp).
bool loadSettings();

// Saves the current TARGET_*/*_DUR_MS globals to EEPROM. Call this only
// when a setting actually changes (e.g. right after '#' commits a new
// value) -- NOT every loop -- to protect EEPROM write endurance.
void saveSettings();

// --- Persisted run state (for resuming after a power outage) --------------
// Checkpoints what phase/mode we're in and how far into the phase we are.
// Call at phase transitions, and periodically (throttled) while running.
void saveRunState(ProcessState state, OperationMode mode, ProcessState manualState,
                   bool timerWasActive, unsigned long phaseElapsedMs);

// Attempts to load a previously saved in-progress run. Returns true if a
// valid saved run was found. outageDurationMs is how long (in ms) the RTC
// thinks passed between the last checkpoint and now -- use it to decide
// whether to resume, extend, or abandon the phase.
bool loadRunState(ProcessState &state, OperationMode &mode, ProcessState &manualState,
                   bool &timerWasActive, unsigned long &phaseElapsedMs,
                   unsigned long &outageDurationMs);

// Invalidates the saved run state. Call when a run finishes normally
// (COMPLETE) or is manually/fault-cleared back to SYSTEM_IDLE, so a
// finished run is never mistaken for an interrupted one on next boot.
void clearRunState();

#endif
