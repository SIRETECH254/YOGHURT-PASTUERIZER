
#ifndef CONFIG_SETTINGS_H
#define CONFIG_SETTINGS_H

// Centralized Pin Configuration Map
#define BUTTON_START_PIN   2   
#define BUTTON_STOP_PIN    3   
#define BUZZER_PIN         6   // Moved off pin 4 to keep 4/5 free per RTC wiring plan

// Hardware Relay Pin Allocations
#define RELAY_HEATER_PIN   7   
#define RELAY_AGITATOR_PIN 8   
#define RELAY_COOLING_PIN  9   

// Dedicated 3-Pin Analog Sensor Allocation Maps
#define ANALOG_HEAT_PIN    A1  // For Heating Jacket Temperature Probe
#define ANALOG_COOL_PIN    A0  // For Inner Cooling Vessel Temperature Probe (Product)

// Keypad Configuration Matrix Pins
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 3

// Graphic LCD 12864B Control Pins
#define DISPLAY_CS_PIN   29  // RS Pin
#define DISPLAY_SID_PIN  30  // R/W Pin
#define DISPLAY_SCLK_PIN 31  // E Pin
#define DISPLAY_RST_PIN  32  // RST Pin

// RTC Module (DS3231) I2C Pins
// NOTE: these are hardware I2C on the Mega 2560 -- fixed at pins 20/21 by
// the chip itself, NOT re-mappable the way a software pin would be.
// Wire.begin() in memory_storage.cpp uses them automatically; the defines
// below are for wiring reference/documentation, consistent with the rest
// of this pin map. Pins 4 and 5 are deliberately left unused by the RTC
// (not needed for its core time/memory function) -- free for something
// else later, e.g. the module's SQW/interrupt pin, if you want it.
#define RTC_SDA_PIN 20
#define RTC_SCL_PIN 21
//   RTC VCC -> 5V
//   RTC GND -> GND
//   RTC SDA -> RTC_SDA_PIN (20)
//   RTC SCL -> RTC_SCL_PIN (21)

// =========================================================================
// SYSTEM RECOVERY & RUNTIME PARAMETERS
// =========================================================================
const float MAX_SAFE_TEMP = 110.0; // Raised to 110C to prevent false over-temp trips during high-heat transfers
const float MIN_SAFE_TEMP = -5.0;  // Open-circuit/unplugged/freezing protection floor
const float HEAT_PROBE_OFFSET = 0.0; // Calibration offset for Jacket Probe
const float COOL_PROBE_OFFSET = 0.0; // Calibration offset for Product Probe

// Global dynamic process memory spaces shared across system frames
extern float TARGET_HEAT_TEMP;
extern unsigned long HEAT_DUR_MS;

extern float TARGET_COOL_TEMP;
extern unsigned long COOL_DUR_MS;

extern unsigned long MIX_DUR_MS;

extern float TARGET_HOLD_TEMP;
extern unsigned long HOLD_DUR_MS;

#endif
