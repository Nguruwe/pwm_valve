#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MAX31865.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================================
// --- НАЗНАЧЕНИЕ ПИНОВ (Строго под ваше железо) ---
// ============================================================================
#define ONE_WIRE_BUS  2   // DS18B20 (Паразитное питание, D2)
#define VALVE_PWM_PIN 3   // ШИМ Соленоида (Timer 2, D3)
#define BUZZER_PIN    4   // Зуммер (D4)
#define LED1_PIN      5   // LED 1: Залёт / Авария (Красный)
#define LED2_PIN      6   // LED 2: Режим работы (Зеленый)
#define LED3_PIN      7   // LED 3: Соленоид открыт (Синий)

// Кнопки: A0, A1, A2, A3, D8
const uint8_t BTN_PINS[5] = {A0, A1, A2, A3, 8};

// --- ФИЛЬТРАЦИЯ И ОПРОС PT1000 ---
unsigned long lastPtReadMs = 0;
const unsigned long PT_READ_INTERVAL_MS = 200; // Опрашиваем Pt1000 раз в 200 мс (5 раз в секунду)

// MAX31865 (Аппаратный SPI: D10 CS, D11 MOSI, D12 MISO, D13 SCK)
#define MAX_CS        10
Adafruit_MAX31865 maxThermo = Adafruit_MAX31865(MAX_CS);
#define RREF          4300.0
#define RNOMINAL      1000.0

// DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dsSensors(&oneWire);
DeviceAddress dsCubeAddr;
bool dsFound = false;

// I2C Дисплей и Барометр (A4 SDA, A5 SCL)
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP085 bmp;
bool bmpOK = false;

// ============================================================================
// --- НАСТРОЙКИ РЕКТИФИКАЦИИ И СОЛЕНОИДА ---
// ============================================================================
const uint8_t PWM_MAX_VAL      = 255;  // 100% ШИМ при форсировании
const unsigned long BOOST_MS   = 150;  // Длительность форс-импульса (мс)
const uint8_t PWM_HOLD_VAL     = 40;   // Ток удержания (~35% мощности)

const float TEMP_ZALET_LIMIT   = 0.10; // Порог залёта (°C)
const float BARO_COEFF         = 0.037;// Поправка Tкип (°C на 1 мм рт. ст.)
const float TEMP_CUBE_STOP     = 98.00;// Завершение отбора по кубу (°C)
const float TEMP_BODY_ALARM    = 50.00;// Перегрев корпуса блока (°C)

const unsigned long WAIT_STABILIZE_MS = 300000; // 5 минут отстоя (300 000 мс)
const unsigned long PWM_PERIOD_MS     = 10000;  // Период ШИМ отбора (10 секунд)

// ============================================================================
// --- ПЕРЕМЕННЫЕ СОСТОЯНИЯ ---
// ============================================================================
enum WorkMode { MANUAL, AUTO, PWM_AUTO };
WorkMode currentMode = MANUAL;

// Измерения
float tempColumn = 0.00; 
float cubeTemp   = -127.0; 
float tempBody   = 0.00; 
float pressmmHg  = 0.00; 

// Базовые уставки
float tempBase  = 0.00;
float pressBase = 0.00;
bool  isBaseSet = false;

// Управление соленоидом
bool          isSolenoidActive = false;
bool          isBoosting       = false;
unsigned long boostStartTime   = 0;

uint8_t       pwmDutyPercent   = 80;    // Стартовая скважность (80%)
unsigned long pwmCycleStartMs  = 0;

// Флаги залёта и таймеры
bool          isZaletActive    = false;
bool          isStabilizing    = false;
unsigned long stabilizeStartMs = 0;
unsigned long lastZaletBeepMs  = 0;

// Счётчик залётов
uint16_t zaletCount = 0; // Счётчик залётов

// Кнопки и Экран
int           lastPressedButton = -1;
uint8_t       currentPage       = 1;
unsigned long pageSwitchMs      = 0;

unsigned long lastDisplayUpdate = 0;
unsigned long lastDSRequestTime = 0;

// ============================================================================
// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
// ============================================================================
void beep(uint16_t freq = 2000, uint16_t duration = 40) {
  tone(BUZZER_PIN, freq, duration);
}

void setupTimer2_Ultrasound() {
  pinMode(VALVE_PWM_PIN, OUTPUT);
  TCCR2B = (TCCR2B & 0xF8) | 0x01; // 31.25 kHz
  analogWrite(VALVE_PWM_PIN, 0);
}

float readPt1000() {
  uint8_t fault = maxThermo.readFault();
  
  if (fault) {
    maxThermo.clearFault();
    
    // Если произошел сбой из-за наводки, перезанициализируем модуль по SPI
    maxThermo.begin(MAX31865_2WIRE); 
    return -999.0; // Сигнал ошибки для фильтра
  }

  float rawTemp = maxThermo.temperature(RNOMINAL, RREF);

  // Фильтрация заведомо нереальных бросков от наводок (например, > 150°C или < -20°C)
  if (rawTemp < -20.0 || rawTemp > 150.0) {
    return -999.0; 
  }

  return rawTemp;
}

// Управление соленоидом (Форсирование -> Удержание)
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

// ============================================================================
// --- SETUP ---
// ============================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  setupTimer2_Ultrasound();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
  }

  maxThermo.begin(MAX31865_2WIRE);

  dsSensors.begin();
  if (dsSensors.getDeviceCount() > 0) {
    if (dsSensors.getAddress(dsCubeAddr, 0)) {
      dsFound = true;
      dsSensors.setResolution(dsCubeAddr, 11);
      dsSensors.setWaitForConversion(false);
      dsSensors.requestTemperatures();
      lastDSRequestTime = millis();
    }
  }

  lcd.init();
  lcd.backlight();
  bmpOK = bmp.begin();

  lcd.setCursor(0, 0);
  lcd.print(bmpOK ? "BMP: OK" : "BMP: ERR");
  lcd.setCursor(9, 0);
  lcd.print(dsFound ? "DS: OK" : "DS: ERR");

  beep(1000, 100);
  delay(1200);
  lcd.clear();
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
      analogWrite(VALVE_PWM_PIN, PWM_HOLD_VAL);
    }
  }

// 2. ЧТЕНИЕ ДАТЧИКОВ
  
  // Опрос Pt1000 ровно раз в 200 мс
  if (currentMillis - lastPtReadMs >= PT_READ_INTERVAL_MS) {
    lastPtReadMs = currentMillis;
    
    float freshTemp = readPt1000();
    if (freshTemp != -999.0) {
      if (tempColumn == 0.0) {
        tempColumn = freshTemp; // Первичная инициализация при старте
      } else {
        // Коэффициент 0.10 при опросе 5 раз в сек дает сглаживание примерно на 2 секунды.
        // Чем меньше число (например, 0.05), тем сильнее и плавнее фильтр.
        tempColumn = (tempColumn * 0.90) + (freshTemp * 0.10);
      }
    }
  }

  if (bmpOK) {
    pressmmHg = bmp.readPressure() * 0.00750062;
    tempBody  = bmp.readTemperature();
  }

  if (dsFound && (currentMillis - lastDSRequestTime >= 500)) {
    cubeTemp = dsSensors.getTempC(dsCubeAddr);
    dsSensors.requestTemperatures();
    lastDSRequestTime = currentMillis;
  }

  // 3. ОБРАБОТКА КНОПОК
  int currentPressed = -1;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(BTN_PINS[i]) == LOW) {
      currentPressed = i + 1;
      break;
    }
  }

if (currentPressed != -1 && currentPressed != lastPressedButton) {
    beep(2400, 30);

    switch (currentPressed) {
      case 1: // Кнопка 1 (A0): Открытие/Закрытие клапана в ручном режиме
        if (currentMode == MANUAL) {
          setSolenoid(!isSolenoidActive);
        }
        break;

      case 2: // Кнопка 2 (A1): Переключение режима
        if (currentMode == MANUAL) currentMode = AUTO;
        else if (currentMode == AUTO) currentMode = PWM_AUTO;
        else currentMode = MANUAL;

        isZaletActive = false;
        isStabilizing = false;
        setSolenoid(false);
        break;

      case 3: // Кнопка 3 (A2): Фиксация T_base и P_base / Сброс аварии и залётов
        tempBase   = tempColumn;
        pressBase  = pressmmHg;
        isBaseSet  = true;
        isZaletActive = false;
        isStabilizing = false;
        zaletCount = 0; // <-- Сбрасываем счётчик при установке базы
        digitalWrite(LED1_PIN, LOW);
        break;

      case 4: // Кнопка 4 (A3): Корректировка скважности ШИМ
        if (pwmDutyPercent > 10) pwmDutyPercent -= 10;
        else pwmDutyPercent = 80;
        break;

      case 5: // Кнопка 5 (D8): Переключение страниц дисплея
        currentPage++;
        if (currentPage > 3) currentPage = 1;
        pageSwitchMs = currentMillis;
        lcd.clear(); // Очищаем экран ТОЛЬКО один раз при смене страницы
        break;
    }
    lastPressedButton = currentPressed;
  } else if (currentPressed == -1) {
    lastPressedButton = -1;
  }

  // 4. ЗАЩИТА: ПЕРЕГРЕВ КОРПУСА И ОКОНЧАНИЕ ОТБОРА ПО КУБУ
  if (bmpOK && tempBody >= TEMP_BODY_ALARM) {
    setSolenoid(false);
    digitalWrite(LED1_PIN, HIGH);
    beep(3500, 100);
  }

  if (dsFound && cubeTemp >= TEMP_CUBE_STOP && currentMode != MANUAL) {
    setSolenoid(false);
    currentMode = MANUAL;
    // Тройная финишная трель
    for (int i = 0; i < 3; i++) { beep(3000, 200); delay(100); }
  }

  // 5. ЛОГИКА АВТОМАТИКИ (AUTO И PWM_AUTO)
  if ((currentMode == AUTO || currentMode == PWM_AUTO) && isBaseSet) {
    digitalWrite(LED2_PIN, HIGH); // Зеленый LED
    float currentTTarget = tempBase + (pressmmHg - pressBase) * BARO_COEFF;

    // Проверка залёта
    if (tempColumn >= (currentTTarget + TEMP_ZALET_LIMIT)) {
      if (!isZaletActive) {
        isZaletActive = true;
        isStabilizing = false;
        zaletCount++; // <-- Увеличиваем счётчик залётов
        digitalWrite(LED1_PIN, HIGH); // Красный LED
        beep(2400, 100);
        setSolenoid(false);

        if (currentMode == PWM_AUTO) {
          if (pwmDutyPercent > 10) pwmDutyPercent -= 10;
          else {
            currentMode = MANUAL; // Скважность упала до нуля — финиш
          }
        }
      }
    } else {
      if (isZaletActive && !isStabilizing) {
        isStabilizing = true;
        stabilizeStartMs = currentMillis;
      }
    }

    // Выдержка 5 минут отстоя
    if (isStabilizing) {
      if (currentMillis - lastZaletBeepMs >= 60000) { // Пик раз в минуту
        beep(2000, 30);
        lastZaletBeepMs = currentMillis;
      }

      if (currentMillis - stabilizeStartMs >= WAIT_STABILIZE_MS) {
        isZaletActive = false;
        isStabilizing = false;
        digitalWrite(LED1_PIN, LOW);
      }
    }

    // Исполнение отбора
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

// 6. ОБНОВЛЕНИЕ ДИСПЛЕЯ (Раз в 300 мс)
  if (currentMillis - lastDisplayUpdate >= 300) {
    lastDisplayUpdate = currentMillis;

    // Автовозврат на Стр 1 через 5 секунд
    if (currentPage != 1 && (currentMillis - pageSwitchMs >= 5000)) {
      currentPage = 1;
      lcd.clear(); // Очищаем экран при автовозврате
    }

    // ВНИМАНИЕ: lcd.clear() отсюда убран!

    switch (currentPage) {
      case 1: // Главный рабочий экран (Строка 1: "Pt:78.4  C:82.1 ")
        lcd.setCursor(0, 0);
        lcd.print("Pt:"); 
        if (tempColumn > -50 && tempColumn < 200) {
          lcd.print(tempColumn, 2);
          if (tempColumn < 100.0) lcd.print(" "); // Пробел для стирания лишнего знака
        } else {
          lcd.print("ERR ");
        }

        lcd.setCursor(8, 0);
        lcd.print(" C:");
        if (dsFound && cubeTemp > -50 && cubeTemp < 125) {
          lcd.print(cubeTemp, 1);
          lcd.print(" ");
        } else {
          lcd.print("ERR ");
        }

        // Строка 2: "MAN     V:OFF  "
        lcd.setCursor(0, 1);
        if (currentMode == MANUAL)        lcd.print("MAN   ");
        else if (currentMode == AUTO)    lcd.print("AUTO  ");
        else {
          lcd.print("P"); 
          lcd.print(pwmDutyPercent); 
          if (pwmDutyPercent < 100) lcd.print("% ");
          else lcd.print("%");
        }

        lcd.setCursor(8, 1);
        if (isBoosting)            lcd.print(" V:BOOST");
        else if (isSolenoidActive) lcd.print(" V:HOLD ");
        else                       lcd.print(" V:OFF  ");
        break;

      case 2: // Экран уставок и залётов
        // --- Строка 1: Tbase и Счётчик залётов ---
        lcd.setCursor(0, 0);
        lcd.print("Tb:");
        if (isBaseSet) {
          lcd.print(tempBase, 2);
        } else {
          lcd.print("NONE ");
        }

        lcd.setCursor(9, 0);
        lcd.print("Z:");
        lcd.print(zaletCount);
        lcd.print("   "); // Пробелы для затирания лишних цифр при сбросе

        // --- Строка 2: Pbase ---
        lcd.setCursor(0, 1);
        lcd.print("Pb:");
        if (isBaseSet) {
          lcd.print(pressBase, 1);
          lcd.print(" ");
        } else {
          lcd.print("NONE ");
        }
        break;

      case 3: // Экран диагностики
        lcd.setCursor(0, 0);
        lcd.print("Pcurr:"); 
        lcd.print(pressmmHg, 1); 
        lcd.print(" mmHg  ");

        lcd.setCursor(0, 1);
        lcd.print("Tin:"); 
        lcd.print(tempBody, 1); 
        lcd.print(" C    ");
        break;
    }
  }
}