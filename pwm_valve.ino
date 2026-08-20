#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ================= НАСТРОЙКИ =================
// --- Дисплей ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const int DISPLAY_UPDATE_INTERVAL = 200; // мс, для плавного мигания

// --- Датчик температуры DS18B20 ---
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
const int TEMP_COMPARE_INTERVAL = 500; // мс, частота сравнения с установкой
unsigned long lastTempRequest = 0;     // Время последнего запроса температуры

// --- Кнопки ---
const int BUTTON1_PIN = 3; // Кнопка 1: Автоматический режим / Перезахват / Показ SP
const int BUTTON2_PIN = 4; // Кнопка 2: Ручной режим (триггер ШИМ)

// --- Светодиод ---
const int LED_PIN = 12; // Внешний светодиод

// --- Настройки ШИМ для соленоида (25 кГц на пине 9) ---
const int SOLENOID_PIN = 9;
const int PULL_VALUE = 319; // 100% мощность
const int HOLD_VALUE = 99;  // Мощность удержания (подбирать под соленоид)
const int PULL_TIME = 300;  // Время полной мощности для втягивания (мс)

// --- Параметры управления ---
const float TEMP_THRESHOLD = 0.125; // Превышение для срабатывания (градусы)
const float TEMP_HYSTERESIS = 0.0;  // Гистерезис

// --- Константы для мигания и отображения SP ---
const unsigned long FLASH_DURATION = 3000;    // Время мигания при включении/перезахвате AUTO (мс)
const unsigned long FLASH_INTERVAL = 500;     // Интервал мигания (полупериод, мс)
const unsigned long SHOW_SP_DURATION = 1000;   // Сколько времени показывать SP при коротком нажатии (мс)
const unsigned long SHOW_SP_INTERVAL = 200;    // Скорость мигания при коротком просмотре (мс)
const unsigned long LONG_PRESS_TIME = 1500;   // Время удержания кнопки для долгого нажатия (мс)

// ================= ПЕРЕМЕННЫЕ СОСТОЯНИЯ =================
float setpointTemp = 0.0;      // Запомненная температура при открытии клапана
float currentTemp = 0.0;       // Текущая температура с датчика
int cycleCounter = 0;          // Счётчик сработок по превышению
bool isSolenoidOn = false;     // true = ШИМ включен (НЗ клапан ОТКРЫТ)
bool isAutoMode = false;       // true = режим автоматического слежения
bool lastButton1State = HIGH;
bool lastButton2State = HIGH;

unsigned long lastDisplayUpdate = 0;
unsigned long lastTempCompare = 0;

// Таймеры для логики отображения и замера кнопок
unsigned long autoModeStartTime = 0;     // Время, когда включили режим AUTO (или перезахватили уставку)
unsigned long showSpStartTime = 0;        // Время, когда запросили показ SP в режиме AUTO
unsigned long button1PressStartTime = 0; // Время, когда кнопка 1 была зажата
bool isButton1Depressed = false;         // Флаг того, что кнопка 1 удерживается
bool longPressExecuted = false;          // Флаг, чтобы долгое нажатие не срабатывало циклически

// ================= НАСТРОЙКА ШИМ (25 кГц) =================
void setupHighFrequencyPWM() {
  pinMode(SOLENOID_PIN, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1);
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
  ICR1 = 319;
  OCR1A = 0;
}

// ================= УПРАВЛЕНИЕ СОЛЕНОИДОМ (НЗ КЛАПАН) =================
void turnSolenoidOn() {
  // Включаем ШИМ -> Нормально Закрытый клапан ОТКРЫВАЕТСЯ
  OCR1A = PULL_VALUE;
  delay(PULL_TIME);
  OCR1A = HOLD_VALUE;
  isSolenoidOn = true;
  digitalWrite(LED_PIN, HIGH);
}

void turnSolenoidOff() {
  // Обесточиваем -> Нормально Закрытый клапан ЗАКРЫВАЕТСЯ
  OCR1A = 0;
  isSolenoidOn = false;
  digitalWrite(LED_PIN, LOW);
}

// ================= ОБНОВЛЕНИЕ ДИСПЛЕЯ (128x32) =================
void updateDisplay() {
  display.clearDisplay();
  unsigned long now = millis();
  
  bool isBlinkingStage = (isAutoMode && (now - autoModeStartTime < FLASH_DURATION));
  bool isShowingSpStage = (isAutoMode && (now - showSpStartTime < SHOW_SP_DURATION));
  
  float tempToDisplay = currentTemp;
  bool shouldRenderTemp = true;

  if (isBlinkingStage) {
    tempToDisplay = setpointTemp;
    if ((now - autoModeStartTime) / FLASH_INTERVAL % 2 == 0) {
      shouldRenderTemp = false; 
    }
  } else if (isShowingSpStage) {
    tempToDisplay = setpointTemp;
    if ((now - showSpStartTime) / SHOW_SP_INTERVAL % 2 == 0) {
      shouldRenderTemp = false; 
    }
  }

  // ---- Верхняя строка: температура ----
  display.setTextColor(SSD1306_WHITE);
  
  if (shouldRenderTemp) {
    display.setTextSize(3);
    display.setCursor(0, 0);
    display.print(tempToDisplay, 2);
    
    display.cp437(true);
    display.setTextSize(1);
    display.setCursor(92, 0);
    display.write(0xF8); // Значок градуса
  }
    
  // ---- Нижняя строка: статус и счётчик ----
  display.setTextSize(1);

  // Статус клапана: для НЗ клапана isSolenoidOn == true означает, что он ОТКРЫТ
  display.setCursor(0, 25);
  display.write(isSolenoidOn ? 25 : 45); // Символ 25 (стрелка/открыто) или 45 (минус/закрыто)
    
  // Режим (AUTO / MANUAL / SP)
  display.print((isBlinkingStage || isShowingSpStage) ? " SP" : (isAutoMode ? " A " : " M "));

  // Счётчик сработок
  display.setCursor(45, 25); 
  display.print("Cnt:");
  display.print(cycleCounter);
  
  display.display();
}

// ================= ОБРАБОТКА КНОПОК =================
void handleButtons() {
  bool b1 = digitalRead(BUTTON1_PIN);
  bool b2 = digitalRead(BUTTON2_PIN);
  unsigned long now = millis();
  
  // ---- Кнопка 1 (AUTO / SP / RE-SETPOINT) ----
  if (b1 == LOW) {
    if (lastButton1State == HIGH) {
      // Момент НАЖАТИЯ кнопки 1
      delay(50); // Антидребезг
      if (digitalRead(BUTTON1_PIN) == LOW) {
        button1PressStartTime = now;
        isButton1Depressed = true;
        longPressExecuted = false;
      }
    } else if (isButton1Depressed && !longPressExecuted) {
      // Кнопка УДЕРЖИВАЕТСЯ
      if (isAutoMode && (now - button1PressStartTime >= LONG_PRESS_TIME)) {
        // --- ДОЛГОЕ НАЖАТИЕ В РЕЖИМЕ AUTO --- (Перезахват температуры)
        turnSolenoidOn();               // Для НЗ клапана: подаем питание, ОТКРЫВАЕМ отбор
        setpointTemp = currentTemp;     // Запоминаем новую ТЕКУЩУЮ температуру
        cycleCounter = 0;               // Сбрасываем счётчик
        autoModeStartTime = now;        // Запускаем мигание
        showSpStartTime = 0;
        longPressExecuted = true;
      }
    }
  } else {
    if (lastButton1State == LOW) {
      // Момент ОТПУСКАНИЯ кнопки 1
      delay(50); // Антидребезг
      if (isButton1Depressed) {
        isButton1Depressed = false;
        
        if (!longPressExecuted) {
          if (!isAutoMode) {
            // --- ПЕРВОЕ НАЖАТИЕ: Вход в режим AUTO ---
            turnSolenoidOn();           // Для НЗ клапана: ОТКРЫВАЕМ отбор
            setpointTemp = currentTemp;
            isAutoMode = true;
            cycleCounter = 0;
            autoModeStartTime = now;
            showSpStartTime = 0;
          } else {
            // --- КОРОТКОЕ НАЖАТИЕ В РЕЖИМЕ AUTO --- (Просмотр уставки)
            showSpStartTime = now;
          }
        }
      }
    }
  }
  
  // ---- Кнопка 2 (MANUAL): триггер ШИМ вкл/выкл ----
  if (b2 == LOW && lastButton2State == HIGH) {
    delay(50);
    if (digitalRead(BUTTON2_PIN) == LOW) {
      if (isSolenoidOn) {
        turnSolenoidOff();
      } else {
        turnSolenoidOn();
      }
      isAutoMode = false; 
      autoModeStartTime = 0;
      showSpStartTime = 0;
      while(digitalRead(BUTTON2_PIN) == LOW);
    }
  }
  
  lastButton1State = b1;
  lastButton2State = b2;
}

// ================= ЛОГИКА АВТОМАТИЧЕСКОГО РЕЖИМА (НЗ КЛАПАН) =================
void handleAutoMode() {
  if (!isAutoMode) return;
  
  // Если отбор открыт (ШИМ подается)
  if (isSolenoidOn) {
    // Температура поднялась выше порога -> Перекрываем отбор (обесточиваем НЗ клапан)
    if (currentTemp >= (setpointTemp + TEMP_THRESHOLD)) {
      turnSolenoidOff();
      cycleCounter++; 
    }
  } else {
    // Если отбор перекрыт, ждем остывания
    // Температура вернулась к норме -> Возобновляем отбор (подаем ШИМ на НЗ клапан)
    if (currentTemp <= (setpointTemp + TEMP_HYSTERESIS)) {
      turnSolenoidOn();
    }
  }
}

// ================= SETUP =================
void setup() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }
  
  sensors.begin();
  sensors.setResolution(12);
  sensors.setWaitForConversion(false); 
  sensors.requestTemperatures();       
  
  setupHighFrequencyPWM();
  
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // При старте система в безопасном состоянии (клапан обесточен / закрыт)
  turnSolenoidOff(); 
  isAutoMode = false;
  cycleCounter = 0;
  setpointTemp = 0.0;
  
  updateDisplay();
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  if (now - lastTempRequest >= 800) {
    currentTemp = sensors.getTempCByIndex(0);
    sensors.requestTemperatures(); 
    lastTempRequest = now;
  }
  
  handleButtons();
  
  if (now - lastTempCompare >= TEMP_COMPARE_INTERVAL) {
    handleAutoMode();
    lastTempCompare = now;
  }
  
  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = now;
  }
}