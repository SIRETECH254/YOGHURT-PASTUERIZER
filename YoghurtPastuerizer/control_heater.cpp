

#include <Arduino.h>
#include "control_heater.h"
#include "config_settings.h"

void initHeater() {
  // Set output pins HIGH before configuration to keep active-low relays from pulsing on boot
  digitalWrite(RELAY_HEATER_PIN, HIGH);   
  digitalWrite(RELAY_AGITATOR_PIN, HIGH); 
  digitalWrite(RELAY_COOLING_PIN, HIGH);  
  digitalWrite(BUZZER_PIN, LOW);          
  
  pinMode(RELAY_HEATER_PIN, OUTPUT);
  pinMode(RELAY_AGITATOR_PIN, OUTPUT);
  pinMode(RELAY_COOLING_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
  
  turnOffAllActuators();
}

void turnOnAgitator() { 
  digitalWrite(RELAY_AGITATOR_PIN, LOW); // LOW turns active-low relay ON
}

void turnOffHeater() { 
  digitalWrite(RELAY_HEATER_PIN, HIGH); // HIGH turns active-low relay OFF
}

void turnOnCoolingValve() { 
  digitalWrite(RELAY_COOLING_PIN, LOW); // LOW turns active-low cooling valve ON
}

void turnOffAllActuators() {
  digitalWrite(RELAY_HEATER_PIN, HIGH);   
  digitalWrite(RELAY_AGITATOR_PIN, HIGH); 
  digitalWrite(RELAY_COOLING_PIN, HIGH);  
  digitalWrite(BUZZER_PIN, LOW);         
}

// -------------------------------------------------------------------------
// Jacket-driven control: the jacket probe responds almost immediately to
// the relay, so the heater is cut off exactly when the JACKET reaches
// target -- no more guessing an early cutoff band based on the slow,
// laggy product probe. The product is left to passively catch up to the
// jacket via conduction after cutoff (phase completion/display still watch
// the product probe separately, in the .ino).
//
// HEAT_HYSTERESIS exists to stop relay chatter: without it, normal probe
// noise right at the exact target causes `error` to bounce back and forth
// across zero, so the relay follows every bounce and clicks rapidly. With
// hysteresis, once the heater turns off at target, it won't turn back on
// until the jacket temp drops a further 1C below target -- a small amount
// of noise can no longer re-trigger it.
// -------------------------------------------------------------------------
const float HEAT_HYSTERESIS = 1.0; // Dead zone below target before the heater is allowed back on

void runPIDControl(float currentTemp, float targetTemp) {
  static bool heaterOn = true; // Heating always starts well below target, so default to ON

  float error = targetTemp - currentTemp;

  if (heaterOn) {
    if (error <= 0) {
      heaterOn = false; // Jacket reached/passed target: stop, let product catch up
    }
  } else {
    if (error > HEAT_HYSTERESIS) {
      heaterOn = true; // Jacket dropped back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Same jacket-driven approach, mirrored for cooling. Same hysteresis logic
// as runPIDControl() above, for the same chatter-prevention reason.
// -------------------------------------------------------------------------
const float COOL_HYSTERESIS = 1.0; // Dead zone above target before the valve is allowed back on

void runCoolingControl(float currentTemp, float targetTemp) {
  static bool coolerOn = true; // Cooling always starts well above target, so default to ON

  float error = currentTemp - targetTemp; // positive while still above target

  if (coolerOn) {
    if (error <= 0) {
      coolerOn = false; // Jacket reached/passed target: stop, let product catch up
    }
  } else {
    if (error > COOL_HYSTERESIS) {
      coolerOn = true; // Jacket rose back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Holding-phase cascade control:
// The product probe is slow to react (milk's thermal mass + jacket-to-product
// lag), so driving the heater/cooler directly off product error -- like the
// old fixed-duration duty-cycle timer did -- lets the jacket bank far more
// heat/cold than the product actually needs before the product probe ever
// reports "close enough". That banked energy keeps transferring into the
// product after cutoff, causing large overshoot/undershoot (e.g. holding at
// 43C, jacket overshoots the product to 51C, then overcorrects down to 40C).
//
// Fix: cascade control, two nested loops:
//   - Outer loop (product probe): turns product error into a JACKET setpoint,
//     biased above/below the holding target and capped at HOLD_JACKET_MARGIN.
//   - Inner loop (jacket probe): drives the heater/cooler off the jacket's
//     OWN temperature vs. that jacket setpoint, with a tight hysteresis. The
//     jacket responds almost immediately to the relay, so this loop can cut
//     power the instant the jacket is satisfied -- it can never bank more
//     energy than the outer loop asked for, no matter how large the initial
//     product-side gap is or what TARGET_HOLD_TEMP is set to.
//
// HOLD_MIN_DWELL_MS is a separate, harder guard on top of the hysteresis
// above: hysteresis alone only rejects noise-sized wobbles, it doesn't cap
// *how often* the relay can flip if the jacket's own thermal response is
// fast enough to swing back through that 0.5C band quickly (e.g. a heating
// element's residual heat carries the jacket a touch past cutoff, then it
// falls back through the band moments later) -- on real hardware this shows
// up as audible relay chatter/clicking ("cracking") and accelerates wear.
// Once the heater or cooler changes state, it's now held there for at least
// HOLD_MIN_DWELL_MS before it's allowed to change again, regardless of how
// the temperature readings move in the meantime.
// -------------------------------------------------------------------------
const float HOLD_JACKET_MARGIN            = 4.0;  // Max jacket bias above/below the holding target
const float HOLD_JACKET_HYSTERESIS        = 0.5;  // Dead zone before the jacket relay is allowed to re-trigger
const float HOLD_DEAD_BAND                = 0.3;  // Product error inside this band -> rest, nothing to correct
const unsigned long HOLD_MIN_DWELL_MS     = 8000; // Minimum time an actuator holds a state before it may switch again

void runHoldingControl(float jacketTemp, float productTemp, float targetTemp) {
  static bool heaterOn = false;
  static bool coolerOn = false;
  static unsigned long lastSwitchTime = 0;

  bool wantHeaterOn = heaterOn;
  bool wantCoolerOn = coolerOn;

  float productError = targetTemp - productTemp; // positive = product under target

  if (abs(productError) <= HOLD_DEAD_BAND) {
    // Product is close enough to target -- rest. Nothing to bank.
    wantHeaterOn = false;
    wantCoolerOn = false;
  } else if (productError > 0) {
    // Product under target: bias the jacket setpoint above target, capped
    float jacketTarget = targetTemp + min(productError, HOLD_JACKET_MARGIN);
    float jacketHeatError = jacketTarget - jacketTemp;
    wantCoolerOn = false;
    if (heaterOn) {
      wantHeaterOn = (jacketHeatError > 0); // Stay on until jacket reaches its (capped) target
    } else {
      wantHeaterOn = (jacketHeatError > HOLD_JACKET_HYSTERESIS); // Only re-trigger past the hysteresis margin
    }
  } else {
    // Product over target: bias the jacket setpoint below target, capped
    float jacketTarget = targetTemp - min(-productError, HOLD_JACKET_MARGIN);
    float jacketCoolError = jacketTemp - jacketTarget;
    wantHeaterOn = false;
    if (coolerOn) {
      wantCoolerOn = (jacketCoolError > 0); // Stay on until jacket reaches its (capped) target
    } else {
      wantCoolerOn = (jacketCoolError > HOLD_JACKET_HYSTERESIS); // Only re-trigger past the hysteresis margin
    }
  }

  // Only actually apply a state change once the minimum dwell time has
  // elapsed since the last one -- caps the max switching rate outright.
  unsigned long now = millis();
  if ((wantHeaterOn != heaterOn || wantCoolerOn != coolerOn) &&
      (now - lastSwitchTime >= HOLD_MIN_DWELL_MS)) {
    heaterOn = wantHeaterOn;
    coolerOn = wantCoolerOn;
    lastSwitchTime = now;
  }

  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}
