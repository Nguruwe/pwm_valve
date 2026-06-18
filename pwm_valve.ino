#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ================= НАСТРОЙКИ =================
// --- Дисплей ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const int DISPLAY_UPDATE_INTERVAL = 500; // мс, константа для обновления дисплея

// --- Датчик температуры DS18B20 ---
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
const int TEMP_COMPARE_INTERVAL = 500; // мс, частота сравнения с установкой

// --- Кнопки ---
const int BUTTON1_PIN = 3; // Кнопка 1: Автоматический режим (открыть и следить)
const int BUTTON2_PIN = 4; // Кнопка 2: Ручной режим (триггер ШИМ)

// --- Светодиод ---
const int LED_PIN = 12; // Внешний светодиод

// --- Настройки ШИМ для соленоида (25 кГц на пине 9) ---
const int SOLENOID_PIN = 9;
const int PULL_VALUE = 319; // 100% мощность
const int HOLD_VALUE = 99;  // Мощность удержания (подбирать под соленоид)
const int PULL_TIME = 300;  // Время полной мощности для втягивания (мс)

// --- Параметры управления ---
const float TEMP_THRESHOLD = 0.1;  // Превышение для срабатывания (градусы)
const float TEMP_HYSTERESIS = 0.0; // Гистерезис (пока 0, подберете позже)

// ================= ПЕРЕМЕННЫЕ СОСТОЯНИЯ =================
float setpointTemp = 0.0;      // Запомненная температура при открытии клапана
float currentTemp = 0.0;       // Текущая температура с датчика
int cycleCounter = 0;          // Счётчик сработок по превышению
bool isSolenoidOn = true;      // true = ШИМ включен (клапан закрыт)
bool isAutoMode = false;       // true = режим автоматического слежения
bool lastButton1State = HIGH;
bool lastButton2State = HIGH;

unsigned long lastDisplayUpdate = 0;
unsigned long lastTempCompare = 0;

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

// ================= УПРАВЛЕНИЕ СОЛЕНОИДОМ =================
void turnSolenoidOn() {
  OCR1A = PULL_VALUE;
  delay(PULL_TIME);
  OCR1A = HOLD_VALUE;
  isSolenoidOn = true;
  digitalWrite(LED_PIN, HIGH);
}

void turnSolenoidOff() {
  OCR1A = 0;
  isSolenoidOn = false;
  digitalWrite(LED_PIN, LOW);
}

// ================= ОБНОВЛЕНИЕ ДИСПЛЕЯ =================
void updateDisplay() {
  display.clearDisplay();
  
  // ---- Верхняя строка: температура (крупный шрифт) ----
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4); // Размер для верхней строки
  display.setCursor(0,0);
  //display.print("T:");
  display.print(currentTemp, 2); // Две сотых
  //display.print("C");

  display.cp437(true); // Включаем режим CP437
  // Теперь можно вывести, например, знак градуса
  display.setTextSize(1);
  display.setCursor(122, 0);
  display.write(0xF8); // Код 0xF8 (248) — это символ "°"
    
  // ---- Нижняя строка: статус и счётчик (шрифт поменьше) ----
  display.setTextSize(2);

  // Статус ШИМ: On или Off
  display.setCursor(0, 50);
  //display.write(isSolenoidOn ? 24 : 25);
  display.write(isSolenoidOn ? 45 : 25);
    
    // Режим (AUTO / MANUAL)
  display.print(isAutoMode ? "A" : "M");

  // Счётчик сработок
  display.setCursor(28, 50);
  display.print("Cnt");
  display.setTextSize(3);
  display.setCursor(68, 44);
  display.print(cycleCounter);
  
  display.display();
}

// ================= ОБРАБОТКА КНОПОК =================
void handleButtons() {
  bool b1 = digitalRead(BUTTON1_PIN);
  bool b2 = digitalRead(BUTTON2_PIN);
  
  // ---- Кнопка 1 (AUTO): открыть клапан и следить ----
  if (b1 == LOW && lastButton1State == HIGH) {
    delay(50); // Антидребезг
    if (digitalRead(BUTTON1_PIN) == LOW) {
      // Если клапан уже открыт, но режим AUTO не активен — всё равно открываем заново
      if (!isAutoMode || isSolenoidOn) {
        // Отключаем ШИМ, открываем клапан
        turnSolenoidOff();
        setpointTemp = currentTemp;       // Запоминаем температуру
        isAutoMode = true;                // Включаем режим AUTO
        cycleCounter = 0;                 // Сбрасываем счётчик (по желанию)
      }
      while(digitalRead(BUTTON1_PIN) == LOW); // Ждём отпускания
    }
  }
  
  // ---- Кнопка 2 (MANUAL): триггер ШИМ вкл/выкл ----
  if (b2 == LOW && lastButton2State == HIGH) {
    delay(50);
    if (digitalRead(BUTTON2_PIN) == LOW) {
      // Переключаем состояние ШИМ (ручной режим)
      if (isSolenoidOn) {
        turnSolenoidOff();
      } else {
        turnSolenoidOn();
      }
      isAutoMode = false; // Выход из AUTO при ручном переключении
      while(digitalRead(BUTTON2_PIN) == LOW);
    }
  }
  
  lastButton1State = b1;
  lastButton2State = b2;
}

// ================= ЛОГИКА АВТОМАТИЧЕСКОГО РЕЖИМА =================
void handleAutoMode() {
  if (!isAutoMode) return;
  
  // Только если клапан открыт (ШИМ выключен) — следим за превышением
  if (!isSolenoidOn) {
    if (currentTemp >= (setpointTemp + TEMP_THRESHOLD)) {
      // Превышение! Закрываем клапан
      turnSolenoidOn();
      cycleCounter++; // Увеличиваем счётчик
    }
  } 
  // Если клапан закрыт (ШИМ включен) — проверяем гистерезис
  else {
    if (currentTemp <= (setpointTemp + TEMP_HYSTERESIS)) {
      // Температура вернулась в допуск — открываем клапан
      turnSolenoidOff();
      // Счётчик НЕ сбрасываем — он накапливается за сессию
    }
  }
}

// ================= SETUP =================
void setup() {
  // --- Инициализация дисплея ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }
  
  // --- Инициализация датчика ---
  sensors.begin();
  sensors.setResolution(12); // Максимальное разрешение (0.0625°C)
  
  // --- Инициализация ШИМ ---
  setupHighFrequencyPWM();
  
  // --- Настройка пинов ---
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // --- Начальное состояние ---
  turnSolenoidOn(); // При старте клапан закрыт (ШИМ включен)
  isAutoMode = false;
  cycleCounter = 0;
  setpointTemp = 0.0;
  
  // Первое обновление дисплея
  updateDisplay();
}

// ================= LOOP =================
void loop() {
  // ---- 1. Чтение температуры (не чаще 1 раза в 750 мс) ----
  sensors.requestTemperatures();
  currentTemp = sensors.getTempCByIndex(0);
  
  // ---- 2. Обработка кнопок (всегда) ----
  handleButtons();
  
  // ---- 3. Логика автоматического режима (если активен) ----
  unsigned long now = millis();
  if (now - lastTempCompare >= TEMP_COMPARE_INTERVAL) {
    handleAutoMode();
    lastTempCompare = now;
  }
  
  // ---- 4. Обновление дисплея (с интервалом 500 мс) ----
  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = now;
  }
  
  // ---- Небольшая задержка для стабильности ----
  delay(100);
}
