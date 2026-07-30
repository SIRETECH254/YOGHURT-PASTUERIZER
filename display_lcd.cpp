

#include <Arduino.h>
#include <U8g2lib.h>
#include "display_lcd.h"
#include "config_settings.h"
#include "states_state_machine.h"
#include "ui_keypad.h"

// Hardware SPI Pin Allocations for standard 12864B graphics module
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, DISPLAY_SCLK_PIN, DISPLAY_SID_PIN, DISPLAY_CS_PIN, DISPLAY_RST_PIN);

void initLCD() { 
  u8g2.begin(); 
}

void drawRunningScreen(const char* title, float currentTemp, float targetTemp, unsigned long totalMs) {
  u8g2.drawStr(2, 11, title);
  u8g2.drawHLine(0, 15, 128);
  
  u8g2.setCursor(4, 27);
  u8g2.print("TEMP: "); u8g2.print(currentTemp, 1); u8g2.print("/"); u8g2.print(targetTemp, 1); u8g2.print("C");
  
  u8g2.setCursor(4, 39);
  u8g2.print("TIME: ");
  char tBuf[9];
  if (!phaseTimerActive) {
    u8g2.print("--:--:--"); 
  } else {
    unsigned long elapsed = millis() - phaseStartTime;
    unsigned long remain = (totalMs > elapsed) ? (totalMs - elapsed) : 0;
    formatTimeStr(tBuf, remain);
    u8g2.print(tBuf);
  }
  
  u8g2.setCursor(4, 51);
  if (!phaseTimerActive) u8g2.print("STATUS: Reaching Temp");
  else u8g2.print("STATUS: Holding Time ");
}

void updateDisplay(float currentTemp, int stateCode) {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 300) return; // Responsive UI repaint loop window
  lastUpdate = millis();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    
    if (currentState == FAULT_ERROR) {
      u8g2.drawStr(2, 11, "!!! SYSTEM FAULT !!!");
      u8g2.drawStr(4, 27, "EMERGENCY STOP ACTIVE");
      u8g2.drawStr(4, 51, "CHECK PROBES/LIMITS");
      continue;
    }

    if (currentMenuState != SCREEN_RUNNING) {
      if (currentMenuState == SCREEN_HOME) {
        u8g2.drawStr(2, 11, "   PASTEURIZER OS");
        u8g2.drawStr(4, 29, "1. AUTO SEQUENCE");
        u8g2.drawStr(4, 41, "2. MANUAL OVERRIDE");
        u8g2.drawStr(4, 53, "3. SET PARAMETERS");
      }
      else if (currentMenuState == SCREEN_AUTO_READY) {
        u8g2.drawStr(2, 11, "   AUTO SEQUENCE");
        u8g2.drawStr(4, 35, "STATUS: READY");
        u8g2.drawStr(4, 60, "[PRESS START TO RUN]");
      }
      else if (currentMenuState == SCREEN_MANUAL_MENU) {
        u8g2.drawStr(2, 11, "   MANUAL SELECT");
        u8g2.drawStr(4, 29, "1: HEAT   2: COOL");
        u8g2.drawStr(4, 45, "3: MIX    4: HOLD");
      }
      else if (currentMenuState == SCREEN_MANUAL_READY) {
        u8g2.drawStr(2, 11, "   MANUAL READY");
        u8g2.setCursor(4, 29);
        u8g2.print("PHASE: ");
        if (selectedManualState == HEATING) u8g2.print("HEATING");
        else if (selectedManualState == COOLING) u8g2.print("COOLING");
        else if (selectedManualState == MIXING) u8g2.print("MIXING");
        else if (selectedManualState == HOLDING) u8g2.print("HOLDING");
        
        u8g2.setCursor(4, 42);
        u8g2.print("CUR TEMP: "); u8g2.print(currentTemp, 1); u8g2.print(" C");
        u8g2.drawStr(4, 60, "[PRESS START TO RUN]");
      }
      else if (currentMenuState == SCREEN_SETTINGS_MAIN) {
        u8g2.drawStr(2, 11, "   EDIT PARAMETERS");
        u8g2.drawStr(4, 29, "1:HEAT VAR 2:COOL VAR");
        u8g2.drawStr(4, 45, "3:MIX VAR  4:HOLD VAR");
      }
      else if (currentMenuState >= SCREEN_SET_HEAT && currentMenuState <= SCREEN_SET_HOLD) {
        u8g2.drawStr(2, 11, "   ENTER NEW VALUE");
        u8g2.setCursor(4, 29);
        if (settingCursor == 0) u8g2.print("TEMP C: ");
        else u8g2.print("HHMMSS: ");
        u8g2.print(inputBuffer);
        u8g2.drawStr(4, 60, "[# to Save  * Back]");
      }
    } 
    else {
      switch((ProcessState)stateCode) {
        case HEATING: drawRunningScreen("   PHASE: HEATING", currentTemp, TARGET_HEAT_TEMP, HEAT_DUR_MS); break;
        case COOLING: drawRunningScreen("   PHASE: COOLING", currentTemp, TARGET_COOL_TEMP, COOL_DUR_MS); break;
        case HOLDING: drawRunningScreen("   PHASE: HOLDING", currentTemp, TARGET_HOLD_TEMP, HOLD_DUR_MS); break;
        
        case MIXING:  
          u8g2.drawStr(2, 11, "   PHASE: MIXING");
          u8g2.drawHLine(0, 15, 128);
          
          u8g2.setCursor(4, 27); 
          u8g2.print("TEMP: "); u8g2.print(currentTemp, 1); u8g2.print(" C");
          
          u8g2.setCursor(4, 39); 
          u8g2.print("TIME: ");
          {
             char tBuf[9];
             unsigned long remain = (MIX_DUR_MS > (millis() - phaseStartTime)) ? (MIX_DUR_MS - (millis() - phaseStartTime)) : 0;
             formatTimeStr(tBuf, remain);
             u8g2.print(tBuf);
          }
          
          u8g2.setCursor(4, 51);
          u8g2.print("STATUS: Agitating ");
          break;
          
        case COMPLETE:
          u8g2.drawHLine(0, 15, 128);
          u8g2.setCursor(2, 11);
          
          if (currentMode == MODE_MANUAL) {
            if (selectedManualState == HEATING) u8g2.print(" HEATING COMPLETED");
            else if (selectedManualState == COOLING) u8g2.print(" COOLING COMPLETED");
            else if (selectedManualState == MIXING) u8g2.print(" MIXING COMPLETED");
            else if (selectedManualState == HOLDING) u8g2.print(" HOLDING COMPLETED");
          } else {
            u8g2.print("  PROCESS COMPLETED");
          }
          
          u8g2.setCursor(4, 32);
          u8g2.print("CURRENT TEMP: "); 
          u8g2.print(currentTemp, 1); 
          u8g2.print(" C");
          
          u8g2.drawStr(4, 56, "[*] TO RETURN BACK");
          break;
      }
    }
  } while (u8g2.nextPage());
}
