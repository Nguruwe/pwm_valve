#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MAX31865.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// ============================================================================
// --- НАЗНАЧЕНИЕ ПИНОВ ---
// ============================================================================
#define ONE_WIRE_BUS  2   // DS18B20 (Паразитное питание, D2)
#define VALVE_PWM_PIN 3   // ШИМ Соленоида (Timer 2, D3)
#define BUZZER_PIN    4   // Зуммер (D4)
#define LED1_PIN      5   // LED 1: Залёт / Авария (Красный)
#define LED2_PIN      6   // LED 2: Режим работы (Зеленый)
#define LED3_PIN      7   // LED 3: Соленоид открыт (Синий)

// Кнопки: A0 (Вправо), A1 (Вверх), A2 (Select), A3 (Вниз), D8 (Влево)
const uint8_t BTN_PINS[5] = {A0, A1, A2, A3, 8};
enum BtnIndex { BTN_RIGHT = 0, BTN_UP = 1, BTN_SELECT = 2, BTN_DOWN = 3, BTN_LEFT = 4 };

// MAX31865 (SPI: D10 CS, D11 MOSI, D12 MISO, D13 SCK)
#define MAX_CS        10
Adafruit_MAX31865 maxThermo = Adafruit_MAX31865(MAX_CS);
#define RREF          4300.0
#define RNOMINAL      1000.0

// DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dsSensors(&oneWire);
DeviceAddress dsCubeAddr;
bool dsFound = false;

// I2C Дисплей и Барометр
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP085 bmp;
bool bmpOK = false;

// ============================================================================
// --- СТРУКТУРА EEPROM И НАСТРОЙКИ ПО УМОЛЧАНИЮ ---
// ============================================================================
#define EEPROM_MAGIC 0x4157 // Сигнатура целостности данных EEPROM

struct EEPROM_Settings {
  uint16_t magic;
  uint8_t  emaAlphaPercent;   // Фильтр Pt1000 (1..50%, default 5%)
  uint8_t  pressAlphaPercent; // Фильтр давления BMP (1..50%, default 3%)
  uint16_t zaletLimitC100;    // Дельта залёта в сотых °C (0.01..0.50 °C, default 10 -> 0.10°C)
  uint8_t  pwmStartDuty;      // Стартовая скважность ШИМ (10..100%, default 80%)
  uint8_t  pwmDecrStep;       // Шаг декремента ШИМ (1..25%, default 10%)
  uint8_t  stabilizeTimeMin;  // Время отстоя в минутах (1..15 min, default 5 min)
  uint8_t  solenoidHoldPwm;   // ШИМ удержания соленоида (10..255, default 40)
  uint16_t cubeStopBodyC10;   // Отсечка куба для AUTO (тело), default 870 (87.0°C)
  uint16_t cubeStopTailC10;   // Отсечка куба для PWM_AUTO (хвосты), default 990 (99.0°C)
};

EEPROM_Settings settings;

void loadDefaultSettings() {
  settings.magic            = EEPROM_MAGIC;
  settings.emaAlphaPercent  = 5;    // Temp Pt1000: 5% (проверено на практике)
  settings.pressAlphaPercent= 3;    // Press BMP: 3% (стабильный порог залёта)
  settings.zaletLimitC100   = 10;   // 0.10 °C
  settings.pwmStartDuty     = 80;
  settings.pwmDecrStep      = 10;
  settings.stabilizeTimeMin = 5;
  settings.solenoidHoldPwm  = 40;
  settings.cubeStopBodyC10  = 870;  // 87.0 °C для тела
  settings.cubeStopTailC10  = 990;  // 99.0 °C для хвостов
}

void saveSettingsToEEPROM() {
  EEPROM.put(0, settings);
}

void initEEPROM() {
  EEPROM.get(0, settings);
  if (settings.magic != EEPROM_MAGIC) {
    loadDefaultSettings();
    saveSettingsToEEPROM();
  }
}

// ============================================================================
// --- КОНСТАНТЫ И ПЕРЕМЕННЫЕ АВТОМАТИКИ ---
// ============================================================================
const uint8_t  PWM_MAX_VAL    = 255;
const unsigned long BOOST_MS  = 150;
const float    BARO_COEFF     = 0.037;  // °C на 1 мм рт. ст.
const float    TEMP_BODY_ALARM= 50.0;   // Перегрев корпуса
const unsigned long PWM_PERIOD_MS = 10000; // Цикл ШИМ 10 сек

enum WorkMode { MANUAL, AUTO, PWM_AUTO };
WorkMode currentMode = MANUAL;

// Измерения
float tempColumn    = 0.00;
float cubeTemp      = -127.0;
float tempBody      = 0.00;
float pressmmHg     = 0.00;
float pressFiltered = 0.00;

// Фильтрация Pt1000
unsigned long lastPtReadMs = 0;
const unsigned long PT_READ_INTERVAL_MS = 200;

// Базовые уставки
float tempBase  = 0.00;
float pressBase = 0.00;
bool  isBaseSet = false;

// Соленоид
bool          isSolenoidActive = false;
bool          isBoosting       = false;
unsigned long boostStartTime   = 0;
uint8_t       pwmDutyPercent   = 80;
unsigned long pwmCycleStartMs  = 0;

// Флаги залётов
bool          isZaletActive    = false;
bool          isStabilizing    = false;
unsigned long stabilizeStartMs = 0;
unsigned long lastZaletBeepMs  = 0;
uint16_t      zaletCount       = 0;

// ============================================================================
// --- СОСТОЯНИЯ UI И УПРАВЛЕНИЯ ЭКРАНАМИ ---
// ============================================================================
enum UiState { VIEW_MODE, EDIT_MODE };
UiState uiState = VIEW_MODE;

uint8_t currentPage = 1;      // 1..12
uint8_t selectedItem = 0;     // 0 — нет выбора, 1..N — выбранный элемент
bool    blinkState = false;   // Для мигания символов
unsigned long lastBlinkMs = 0;
unsigned long lastActivityMs = 0; // Для таймаутов отмены и автовозврата Home

// Переменная черновика для редактирования
int32_t draftValue = 0;
bool    confirmBlink = false; // Быстрое мигание при приеме
unsigned long confirmStartMs = 0;

// Опрос кнопок
int  lastPressedBtn = -1;
unsigned long btnHoldStartMs = 0;
bool isAutoRepeat = false;

// ============================================================================
// --- HELPERS (ВЫЧИСЛЕНИЯ И КОНВЕРТАЦИЯ) ---
// ============================================================================
inline float getBaroOffset() {
  if (!isBaseSet) return 0.0;
  return (pressmmHg - pressBase) * BARO_COEFF;
}

inline float getTargetTemp() { return tempBase + getBaroOffset(); }

inline float getZaletLimit() { return settings.zaletLimitC100 / 100.0f; }
inline float getZaletThresholdTemp() { return getTargetTemp() + getZaletLimit(); }

inline float getDraftAsFloat() { return draftValue / 100.0f; }

inline float getEmaAlpha() { return settings.emaAlphaPercent / 100.0f; }
inline float getPressAlpha() { return settings.pressAlphaPercent / 100.0f; }

inline float getCubeStopBody() { return settings.cubeStopBodyC10 / 10.0f; }
inline float getCubeStopTail() { return settings.cubeStopTailC10 / 10.0f; }

// ============================================================================
// --- ЗВУКОВЫЕ СИГНАЛЫ ---
// ============================================================================

void beep(uint16_t freq = 2000, uint16_t duration = 40) {
  long periodUs = 1000000L / freq;
  long cycles = ((long)freq * duration) / 1000L;
  for (long i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(periodUs / 2);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(periodUs / 2);
  }
}

void soundClick()   { beep(2400, 20); }
void soundConfirm() { beep(2800, 50); delay(60); beep(3200, 70); }
void soundCancel()  { beep(1200, 150); }
void soundEmergency() { beep(1000, 400); }

// ============================================================================
// --- УПРАВЛЕНИЕ АППАРАТУРОЙ ---
// ============================================================================

void setupTimer2_Ultrasound() {
  pinMode(VALVE_PWM_PIN, OUTPUT);
  TCCR2B = (TCCR2B & 0xF8) | 0x01; // 31.25 kHz ШИМ (бесшумный)
  analogWrite(VALVE_PWM_PIN, 0);
}

float readPt1000() {
  uint8_t fault = maxThermo.readFault();
  if (fault) {
    maxThermo.clearFault();
    maxThermo.begin(MAX31865_4WIRE);
    return -999.0;
  }
  float rawTemp = maxThermo.temperature(RNOMINAL, RREF);
  if (rawTemp < -20.0 || rawTemp > 150.0) return -999.0;
  return rawTemp;
}

void setSolenoid(bool active) {
  if (active && !isSolenoidActive) {
    isSolenoidActive = true;
    isBoosting       = true;
    boostStartTime   = millis();
    analogWrite(VALVE_PWM_PIN, PWM_MAX_VAL);
    digitalWrite(LED3_PIN, HIGH);
  } else if (!active && isSolenoidActive) {
    isSolenoidActive = false;
    isBoosting       = false;
    analogWrite(VALVE_PWM_PIN, 0);
    digitalWrite(LED3_PIN, LOW);
  }
}

// Получить количество редактируемых элементов на странице
uint8_t getItemCountForPage(uint8_t page) {
  switch (page) {
    case 1:  return 3; // 1: T_base, 2: Delta T, 3: Mode
    case 2:  return 0; // Информационная
    case 3:  return 1; // EMA Filter Temp Alpha
    case 4:  return 1; // EMA Filter Press Alpha
    case 5:  return 1; // Delta T Limit
    case 6:  return 1; // Start Duty
    case 7:  return 1; // Decr Step
    case 8:  return 1; // Stabilize Time
    case 9:  return 1; // Solenoid Hold PWM
    case 10: return 1; // Cube Body Stop
    case 11: return 1; // Cube Tail Stop
    case 12: return 1; // Factory Reset
    default: return 0;
  }
}

// Загрузить значение элемента в черновик
void loadDraftValue() {
  switch (currentPage) {
    case 1:
      if (selectedItem == 1) draftValue = 0; // Захват базовых T и P
      else if (selectedItem == 2) {
        draftValue = (int32_t)lround(getZaletThresholdTemp() * 100.0);
      }
      else if (selectedItem == 3) draftValue = (int32_t)currentMode;
      break;
    case 3:  draftValue = settings.emaAlphaPercent; break;
    case 4:  draftValue = settings.pressAlphaPercent; break;
    case 5:  draftValue = settings.zaletLimitC100; break;
    case 6:  draftValue = settings.pwmStartDuty; break;
    case 7:  draftValue = settings.pwmDecrStep; break;
    case 8:  draftValue = settings.stabilizeTimeMin; break;
    case 9:  draftValue = settings.solenoidHoldPwm; break;
    case 10: draftValue = settings.cubeStopBodyC10; break;
    case 11: draftValue = settings.cubeStopTailC10; break;
    case 12: draftValue = 0; break;
  }
}

// Сохранить черновик в рабочие переменные и EEPROM
void applyDraftValue() {
  switch (currentPage) {
    case 1:
      if (selectedItem == 1) { // Захват по факту
        tempBase = tempColumn;
        pressBase = pressmmHg;
        isBaseSet = true;
        isZaletActive = false;
        isStabilizing = false;
        zaletCount = 0;
        digitalWrite(LED1_PIN, LOW);
      } else if (selectedItem == 2) {
        float newTarget = draftValue / 100.0f;
        // Из нового порога вычитаем поправку давления и дельту залёта:
        tempBase = newTarget - getBaroOffset() - getZaletLimit();
        isBaseSet = true;
      } else if (selectedItem == 3) {
        currentMode = (WorkMode)draftValue;
        isZaletActive = false;
        isStabilizing = false;
        setSolenoid(false);
        if (currentMode == PWM_AUTO) {
          pwmDutyPercent = settings.pwmStartDuty;
          pwmCycleStartMs = millis();
        }
      }
      break;
    case 3:
      settings.emaAlphaPercent = constrain(draftValue, 1, 50);
      saveSettingsToEEPROM();
      break;
    case 4:
      settings.pressAlphaPercent = constrain(draftValue, 1, 50);
      saveSettingsToEEPROM();
      break;
    case 5:
      settings.zaletLimitC100 = constrain(draftValue, 1, 50);
      saveSettingsToEEPROM();
      break;
    case 6:
      settings.pwmStartDuty = constrain(draftValue, 10, 100);
      pwmDutyPercent = settings.pwmStartDuty;
      saveSettingsToEEPROM();
      break;
    case 7:
      settings.pwmDecrStep = constrain(draftValue, 1, 25);
      saveSettingsToEEPROM();
      break;
    case 8:
      settings.stabilizeTimeMin = constrain(draftValue, 1, 15);
      saveSettingsToEEPROM();
      break;
    case 9:
      settings.solenoidHoldPwm = constrain(draftValue, 10, 255);
      saveSettingsToEEPROM();
      break;
    case 10:
      settings.cubeStopBodyC10 = constrain(draftValue, 300, 1000);
      saveSettingsToEEPROM();
      break;
    case 11:
      settings.cubeStopTailC10 = constrain(draftValue, 300, 1000);
      saveSettingsToEEPROM();
      break;
    case 12:
      if (draftValue == 1) {
        isBaseSet = false;
        isZaletActive = false;
        setSolenoid(false);
        loadDefaultSettings();
        saveSettingsToEEPROM();
        pwmDutyPercent = settings.pwmStartDuty;
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("RESET COMPLETED!");
        soundConfirm();
        delay(1500);
        currentPage = 1;
      }
      break;
  }
}

// Изменение значения черновика стрелками Влево/Вправо
void modifyDraft(int delta) {
  switch (currentPage) {
    case 1:
      if (selectedItem == 2) {
        draftValue = constrain(draftValue + delta, 0, 10000);
      }
      else if (selectedItem == 3) {
        draftValue += delta;
        if (draftValue > 2) draftValue = 0;
        if (draftValue < 0) draftValue = 2;
      }
      break;
    case 3: draftValue = constrain(draftValue + delta, 1, 50); break;
    case 4: draftValue = constrain(draftValue + delta, 1, 50); break;
    case 5: draftValue = constrain(draftValue + delta, 1, 50); break;
    case 6: draftValue = constrain(draftValue + (delta * 5), 10, 100); break;
    case 7: draftValue = constrain(draftValue + delta, 1, 25); break;
    case 8: draftValue = constrain(draftValue + delta, 1, 15); break;
    case 9: draftValue = constrain(draftValue + (delta * 5), 10, 255); break;
    case 10: draftValue = constrain(draftValue + (delta * 5), 300, 1000); break;
    case 11: draftValue = constrain(draftValue + (delta * 5), 300, 1000); break;
    case 12: draftValue = (draftValue == 0) ? 1 : 0; break;
  }
}

// ============================================================================
// --- SETUP ---
// ============================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  initEEPROM();
  pwmDutyPercent = settings.pwmStartDuty;

  setupTimer2_Ultrasound();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
  }

  maxThermo.begin(MAX31865_4WIRE);

  lcd.init();
  lcd.backlight();
  bmpOK = bmp.begin();

  lcd.setCursor(0, 0); lcd.print(bmpOK ? "BMP: OK " : "BMP: ERR");
  lcd.setCursor(0, 1); lcd.print("DS: WAIT");

  soundConfirm();
  delay(1000);
  lcd.clear();
  
  lastActivityMs = millis();
}

// ============================================================================
// --- MAIN LOOP ---
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // 1. АВТОПЕРЕХОД ИЗ ФОРСИРОВАНИЯ В УДЕРЖАНИЕ СОЛЕНОИДА
  if (isSolenoidActive && isBoosting) {
    if (currentMillis - boostStartTime >= BOOST_MS) {
      isBoosting = false;
      analogWrite(VALVE_PWM_PIN, settings.solenoidHoldPwm);
    }
  }

  // 2. ЧТЕНИЕ ДАТЧИКОВ И ФИЛЬТРАЦИЯ
  // Pt1000
  if (currentMillis - lastPtReadMs >= PT_READ_INTERVAL_MS) {
    lastPtReadMs = currentMillis;
    float freshTemp = readPt1000();
    if (freshTemp != -999.0) {
      if (tempColumn == 0.0) tempColumn = freshTemp;
      else {
        tempColumn = (tempColumn * (1.0f - getEmaAlpha())) + (freshTemp * getEmaAlpha());
      }
    }
  }

  // BMP180 / BMP080 (Давление)
  if (bmpOK) {
    float rawPressmmHg = bmp.readPressure() * 0.00750062f;
    tempBody = bmp.readTemperature();

    static bool isPressInit = false; // Флаг: было ли уже первое чтение?

    if (!isPressInit) {
      pressFiltered = rawPressmmHg;  // При первом старте просто берем сырое значение
      isPressInit = true;            // Запоминаем, что инициализация прошла
    } else {
      // В остальные разы спокойно сглаживаем по EMA
      pressFiltered = (pressFiltered * (1.0f - getPressAlpha())) + (rawPressmmHg * getPressAlpha());
    }
    pressmmHg = pressFiltered;
  }

  static unsigned long lastDsReadMs = 0;
  if (currentMillis - lastDsReadMs >= 750) {
    lastDsReadMs = currentMillis;
  
    if (!dsFound) {
      dsSensors.begin();
      if (dsSensors.getDeviceCount() > 0 && dsSensors.getAddress(dsCubeAddr, 0)) {
        dsFound = true;
        dsSensors.setResolution(dsCubeAddr, 11);
        dsSensors.setWaitForConversion(false);
        dsSensors.requestTemperatures();
      }
    } else {
      float rawCube = dsSensors.getTempC(dsCubeAddr);
      if (rawCube > -55.0 && rawCube < 125.0) {
        cubeTemp = rawCube;
      } else {
        dsFound = false; 
      }
      dsSensors.requestTemperatures();
    }
  }

  // 3. ТАЙМАУТЫ, МИГАНИЕ И АВТОВОЗВРАТ HOME
  if (currentMillis - lastBlinkMs >= (confirmBlink ? 100 : 350)) {
    lastBlinkMs = currentMillis;
    blinkState = !blinkState;
  }

  if (confirmBlink && (currentMillis - confirmStartMs >= 1000)) {
    confirmBlink = false;
    uiState = VIEW_MODE;
    selectedItem = 0;
    lcd.clear();
  }

  // Таймаут отмены редактирования (10 сек)
  if (uiState == EDIT_MODE && !confirmBlink) {
    if (currentMillis - lastActivityMs >= 10000) {
      soundCancel();
      uiState = VIEW_MODE;
      selectedItem = 0;
      lcd.clear();
    }
  }

  // Таймаут автовозврата на Главный экран (30 сек бездействия)
  if (uiState == VIEW_MODE && currentPage != 1) {
    if (currentMillis - lastActivityMs >= 30000) {
      currentPage = 1;
      selectedItem = 0;
      soundClick();
      lcd.clear();
    }
  }

  // 4. ОБРАБОТКА КНОПОК
  int activeBtn = -1;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(BTN_PINS[i]) == LOW) {
      activeBtn = i;
      break;
    }
  }

  if (activeBtn != -1) {
    lastActivityMs = currentMillis;

    if (activeBtn != lastPressedBtn) {
      lastPressedBtn = activeBtn;
      btnHoldStartMs = currentMillis;
      isAutoRepeat   = false;

      soundClick();

      if (uiState == VIEW_MODE) {
        if (activeBtn == BTN_RIGHT) {
          currentPage = (currentPage >= 12) ? 1 : currentPage + 1; // <--- ИСПРАВЛЕНО (12 вместо 11)
          lcd.clear();
        } else if (activeBtn == BTN_LEFT) {
          currentPage = (currentPage <= 1) ? 12 : currentPage - 1; // <--- ИСПРАВЛЕНО (12 вместо 11)
          lcd.clear();
        } else if (activeBtn == BTN_DOWN || activeBtn == BTN_UP) {
          uint8_t count = getItemCountForPage(currentPage);
          if (count > 0) {
            uiState = EDIT_MODE;
            selectedItem = (activeBtn == BTN_DOWN) ? 1 : count;
            loadDraftValue();
          }
        } else if (activeBtn == BTN_SELECT) {
          if (currentPage == 1 && currentMode == MANUAL) {
            setSolenoid(!isSolenoidActive);
          }
        }
      } else if (uiState == EDIT_MODE && !confirmBlink) {
        uint8_t count = getItemCountForPage(currentPage);

        if (activeBtn == BTN_DOWN) {
          if (selectedItem < count) {
            selectedItem++;
            loadDraftValue();
          } else {
            soundCancel();
            uiState = VIEW_MODE;
            selectedItem = 0;
            lcd.clear();
          }
        } else if (activeBtn == BTN_UP) {
          if (selectedItem > 1) {
            selectedItem--;
            loadDraftValue();
          } else {
            soundCancel();
            uiState = VIEW_MODE;
            selectedItem = 0;
            lcd.clear();
          }
        } else if (activeBtn == BTN_RIGHT) {
          modifyDraft(1);
        } else if (activeBtn == BTN_LEFT) {
          modifyDraft(-1);
        } else if (activeBtn == BTN_SELECT) {
          soundConfirm();
          applyDraftValue();
          confirmBlink = true;
          confirmStartMs = currentMillis;
        }
      }
    } else {

      if (uiState == EDIT_MODE && !confirmBlink) {
        if (currentMillis - btnHoldStartMs >= (isAutoRepeat ? 100 : 500)) {
          btnHoldStartMs = currentMillis;
          isAutoRepeat = true;
          if (activeBtn == BTN_RIGHT) modifyDraft(1);
          else if (activeBtn == BTN_LEFT) modifyDraft(-1);
        }
      }

      
    }
  } else {
    lastPressedBtn = -1;
  }

  // 5. ЛОГИКА АВТОМАТИКИ
  if (bmpOK && tempBody >= TEMP_BODY_ALARM) {
    setSolenoid(false);
    digitalWrite(LED1_PIN, HIGH);
    soundEmergency();
  }

  if (dsFound && currentMode != MANUAL) {
    float stopLimit = (currentMode == AUTO) ? getCubeStopBody() : getCubeStopTail();
    if (cubeTemp >= stopLimit) {
      setSolenoid(false);
      currentMode = MANUAL;
      soundConfirm();
    }
  }

  if ((currentMode == AUTO || currentMode == PWM_AUTO) && isBaseSet) {
    digitalWrite(LED2_PIN, HIGH);

    if (tempColumn >= getZaletThresholdTemp()) {
      if (!isZaletActive) {
        isZaletActive = true;
        isStabilizing = false;
        zaletCount++;
        digitalWrite(LED1_PIN, HIGH);
        beep(2400, 100);
        setSolenoid(false);

        if (currentMode == PWM_AUTO) {
          if (pwmDutyPercent > settings.pwmDecrStep) {
            pwmDutyPercent -= settings.pwmDecrStep;
          } else {
            currentMode = MANUAL; // Завершение отбора
          }
        }
      }
    } else {
      if (isZaletActive && !isStabilizing) {
        isStabilizing = true;
        stabilizeStartMs = currentMillis;
      }
    }

    if (isStabilizing) {
      if (currentMillis - lastZaletBeepMs >= 60000) {
        beep(2000, 30);
        lastZaletBeepMs = currentMillis;
      }

      if (currentMillis - stabilizeStartMs >= (settings.stabilizeTimeMin * 60000UL)) {
        isZaletActive = false;
        isStabilizing = false;
        pwmCycleStartMs = currentMillis;
        digitalWrite(LED1_PIN, LOW);
      }
    }

    if (!isZaletActive && !isStabilizing) {
      if (currentMode == AUTO) {
        setSolenoid(true);
      } else if (currentMode == PWM_AUTO) {
        unsigned long currentPwmMs = currentMillis - pwmCycleStartMs;
        if (currentPwmMs >= PWM_PERIOD_MS) {
          pwmCycleStartMs = currentMillis;
          currentPwmMs = 0;
        }
        unsigned long openTimeMs = (PWM_PERIOD_MS * pwmDutyPercent) / 100;
        setSolenoid(currentPwmMs < openTimeMs);
      }
    }
  } else {
    digitalWrite(LED2_PIN, LOW);
  }

  // 6. ОТРИСОВКА ЭКРАНА LCD
  lcd.setCursor(0, 0);

  switch (currentPage) {
    case 1: {
      if (uiState == EDIT_MODE && selectedItem == 1 && !blinkState) {
        lcd.print("      ");
      } else {
        if (tempColumn < 100.0) lcd.print(" ");
        lcd.print(tempColumn, 2);
      }
      lcd.print("/");
      if (uiState == EDIT_MODE && selectedItem == 2 && !blinkState) {
        lcd.print("     ");
      } else {
        if (uiState == EDIT_MODE && selectedItem == 2) {
          float draftTarget = getDraftAsFloat();
          lcd.print(draftTarget, 2);
        } else if (isBaseSet) {
          //lcd.print(getTargetTemp(), 2);
          lcd.print(getZaletThresholdTemp(), 2);
        } else {
          lcd.print("--.--");
        }
      }
      lcd.print(isBaseSet ? "C OK" : "C NO");
      
      lcd.setCursor(0, 1);
      lcd.print("C:");
      if (dsFound) {
        if (cubeTemp >= 0.0f && cubeTemp < 100.0f) lcd.print(" "); // выравнивание
          if (cubeTemp > -50.0f && cubeTemp < 125.0f) {
            lcd.print(cubeTemp, 1);
          } else {
            lcd.print("ERR ");
          }
        } else {
        lcd.print("NoDS");
      }

      lcd.setCursor(8, 1);
      if (uiState == EDIT_MODE && selectedItem == 3 && !blinkState) {
        lcd.print("    ");
      } else {
        WorkMode m = (uiState == EDIT_MODE && selectedItem == 3) ? (WorkMode)draftValue : currentMode;
        if (m == MANUAL)     lcd.print("MAN ");
        else if (m == AUTO)  lcd.print("AUTO");
        else {
          lcd.print("P"); lcd.print(pwmDutyPercent); lcd.print("%");
          if (pwmDutyPercent < 100) lcd.print(" ");
        }
      }

      lcd.setCursor(13, 1);
      if (isBoosting)            lcd.print("BST");
      else if (isSolenoidActive) lcd.print("ON ");
      else                       lcd.print("OFF");
      break;
    }

    case 2:
      lcd.print("P:"); lcd.print(pressmmHg, 1); lcd.print("/");
      if (isBaseSet) lcd.print(pressBase, 1);
      else lcd.print("---.-");
      lcd.print(" ");

      lcd.setCursor(0, 1);
      lcd.print("Tin:"); lcd.print(tempBody, 1); lcd.print("C ");
      lcd.print(dsFound ? "DS:OK " : "DS:ERR");
      break;

    case 3:
      lcd.print("EMA Temp Filter ");
      lcd.setCursor(0, 1);
      lcd.print("Weight A_T: ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("   ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : settings.emaAlphaPercent;
        if (v < 10) lcd.print(" ");
        lcd.print(v); lcd.print("%");
      }
      lcd.print("   ");
      break;

    case 4:
      lcd.print("EMA Press Filter");
      lcd.setCursor(0, 1);
      lcd.print("Weight A_P: ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("   ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : settings.pressAlphaPercent;
        if (v < 10) lcd.print(" ");
        lcd.print(v); lcd.print("%");
      }
      lcd.print("   ");
      break;

    case 5:
      lcd.print("Zalet Limit     ");
      lcd.setCursor(0, 1);
      lcd.print("Delta T: ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("    ");
      else {
        float v = (uiState == EDIT_MODE) ? getDraftAsFloat() : getZaletLimit();
        lcd.print(v, 2); lcd.print(" C");
      }
      lcd.print("  ");
      break;

    case 6:
      lcd.print("PWM Start Duty  ");
      lcd.setCursor(0, 1);
      lcd.print("Duty Val: ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("   ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : settings.pwmStartDuty;
        if (v < 100) lcd.print(" ");
        lcd.print(v); lcd.print("%");
      }
      lcd.print("   ");
      break;

    case 7:
      lcd.print("PWM Decr Step   ");
      lcd.setCursor(0, 1);
      lcd.print("Step Val: ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("   ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : settings.pwmDecrStep;
        if (v < 10) lcd.print(" ");
        lcd.print(v); lcd.print("%");
      }
      lcd.print("   ");
      break;

    case 8:
      lcd.print("Stabilize Time  ");
      lcd.setCursor(0, 1);
      lcd.print("Pause:    ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("  ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : settings.stabilizeTimeMin;
        if (v < 10) lcd.print(" ");
        lcd.print(v);
      }
      lcd.print(" min  ");
      break;

    case 9:
      lcd.print("Solenoid Hold   ");
      lcd.setCursor(0, 1);
      lcd.print("Hold PWM: ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("   ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : settings.solenoidHoldPwm;
        if (v < 100) lcd.print(" ");
        if (v < 10)  lcd.print(" ");
        lcd.print(v);
      }
      lcd.print("   ");
      break;

    case 10:
      lcd.print("Cube Body Stop  ");
      lcd.setCursor(0, 1);
      lcd.print("T_body:  ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("    ");
      else {
        float v = (uiState == EDIT_MODE) ? (draftValue / 10.0f) : getCubeStopBody();
        lcd.print(v, 1); lcd.print(" C");
      }
      lcd.print("  ");
      break;

    case 11:
      lcd.print("Cube Tail Stop  ");
      lcd.setCursor(0, 1);
      lcd.print("T_tail:  ");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("    ");
      else {
        float v = (uiState == EDIT_MODE) ? (draftValue / 10.0f) : getCubeStopTail();
        lcd.print(v, 1); lcd.print(" C");
      }
      lcd.print("  ");
      break;

    case 12:
      lcd.print("Factory Reset   ");
      lcd.setCursor(0, 1);
      lcd.print("Press SEL [");
      if (uiState == EDIT_MODE && !blinkState) lcd.print("   ");
      else {
        int v = (uiState == EDIT_MODE) ? draftValue : 0;
        lcd.print(v == 1 ? "YES" : "NO ");
      }
      lcd.print("] ");
      break;
  }
}