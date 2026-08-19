#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MAX31865.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- ПИНЫ УПРАВЛЕНИЯ ---
#define ONE_WIRE_BUS  2   // DS18B20 (Паразитное питание, подтяжка 2.2к на +5V)
#define VALVE_PWM_PIN 3   // ШИМ Соленоида (Timer 2, 31.25 кГц)
#define BUZZER_PIN    4   // Зуммер
#define LED1_PIN      5   // Светодиод 1
#define LED2_PIN      6   // Светодиод 2
#define LED3_PIN      7   // Светодиод 3

// --- КНОПКИ (A0 - A3, D8) ---
const uint8_t BTN_PINS[5] = {A0, A1, A2, A3, 8};

// --- АППАРАТНЫЙ SPI ДЛЯ MAX31865 (D10 CS, D11 MOSI, D12 MISO, D13 SCK) ---
#define MAX_CS        10
Adafruit_MAX31865 maxThermo = Adafruit_MAX31865(MAX_CS);
#define RREF          4300.0
#define RNOMINAL      1000.0

// --- ДС18B20 НА D2 ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dsSensors(&oneWire);
DeviceAddress dsCubeAddr;
bool dsFound = false;

// --- I2C ДИСПЛЕЙ И БАРОМЕТР (A4 SDA, A5 SCL) ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP085 bmp;
bool bmpOK = false;

// --- НАСТРОЙКИ СОЛЕНОИДА ---
const uint8_t PWM_MAX_VAL      = 255; // 100% ШИМ при форсировании
const unsigned long BOOST_MS   = 150; // Длительность форс-импульса (мс)
const uint8_t PWM_HOLD_VAL     = 40;  // Ток удержания (~35% мощности)

bool isSolenoidActive = false;
bool isBoosting       = false;
unsigned long boostStartTime = 0;

int lastPressedButton = -1;
unsigned long lastDisplayUpdate = 0;
unsigned long lastDSRequestTime = 0;
float cubeTemp = -127.0;

// Озвучка нажатия
void beep(uint16_t freq = 2000, uint16_t duration = 40) {
  tone(BUZZER_PIN, freq, duration);
}

// Перевод Таймера 2 на ультразвуковую частоту 31.25 кГц на пине D3
void setupTimer2_Ultrasound() {
  pinMode(VALVE_PWM_PIN, OUTPUT);
  TCCR2B = (TCCR2B & 0xF8) | 0x01; // Предделитель = 1 (16MHz / 512 = 31.25 kHz)
  analogWrite(VALVE_PWM_PIN, 0);
}

// Запрос температуры Pt1000 с подавлением одиночных ошибок FAULT
float readPt1000() {
  uint8_t fault = maxThermo.readFault();
  if (fault) {
    maxThermo.clearFault();
  }
  return maxThermo.temperature(RNOMINAL, RREF);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  setupTimer2_Ultrasound();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  // Настройка кнопок
  for (int i = 0; i < 5; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
  }

  // MAX31865 (Аппаратный SPI)
  maxThermo.begin(MAX31865_2WIRE);

  // DS18B20 (Паразитное питание)
  dsSensors.begin();
  if (dsSensors.getDeviceCount() > 0) {
    if (dsSensors.getAddress(dsCubeAddr, 0)) {
      dsFound = true;
      dsSensors.setResolution(dsCubeAddr, 11); // 11 бит (время замера ~375 мс)
      dsSensors.setWaitForConversion(false);   // Асинхронный опрос
      dsSensors.requestTemperatures();
      lastDSRequestTime = millis();
    }
  }

  // LCD1602 и BMP180
  lcd.init();
  lcd.backlight();
  bmpOK = bmp.begin();

  lcd.setCursor(0, 0);
  lcd.print(bmpOK ? "BMP: OK" : "BMP: ERR");
  lcd.setCursor(9, 0);
  lcd.print(dsFound ? "DS: OK" : "DS: ERR");

  beep(1000, 100); // Звуковой сигнал успешного старта
  delay(1200);
  lcd.clear();

  Serial.println("--- ТЕСТ СИСТЕМЫ ЗАПУЩЕН ---");
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. ОПРОС КНОПОК
  int currentPressed = -1;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(BTN_PINS[i]) == LOW) {
      currentPressed = i + 1;
      break;
    }
  }

  // 2. ОБРАБОТКА НАЖАТИЙ КНОПОК
  if (currentPressed != -1 && currentPressed != lastPressedButton) {
    beep(2400, 30);
    Serial.print("Нажата кнопка: "); Serial.println(currentPressed);

    switch (currentPressed) {
      case 1: // Кнопка 1 (A0): Переключить LED 1
        digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));
        break;
      case 2: // Кнопка 2 (A1): Переключить LED 2
        digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
        break;
      case 3: // Кнопка 3 (A2): Переключить LED 3
        digitalWrite(LED3_PIN, !digitalRead(LED3_PIN));
        break;
      case 4: // Кнопка 4 (A3): ЗАКРЫТЬ СОЛЕНОИД
        isSolenoidActive = false;
        isBoosting       = false;
        analogWrite(VALVE_PWM_PIN, 0);
        Serial.println("Соленоид: ВЫКЛ (0% ШИМ)");
        break;
      case 5: // Кнопка 5 (D8): ОТКРЫТЬ СОЛЕНОИД (Форсирование -> Удержание)
        isSolenoidActive = true;
        isBoosting       = true;
        boostStartTime   = currentMillis;
        analogWrite(VALVE_PWM_PIN, PWM_MAX_VAL);
        Serial.println("Соленоид: ФОРСИРОВАНИЕ (100% ШИМ)");
        break;
    }
    lastPressedButton = currentPressed;
  } else if (currentPressed == -1) {
    lastPressedButton = -1;
  }

  // 3. АВТОПЕРЕХОД ИЗ ФОРСИРОВАНИЯ В УДЕРЖАНИЕ СОЛЕНОИДА
  if (isSolenoidActive && isBoosting) {
    if (currentMillis - boostStartTime >= BOOST_MS) {
      isBoosting = false;
      analogWrite(VALVE_PWM_PIN, PWM_HOLD_VAL);
      Serial.print("Соленоид: УДЕРЖАНИЕ (ШИМ = ");
      Serial.print(PWM_HOLD_VAL);
      Serial.println("/255)");
    }
  }

  // 4. АСИНХРОННЫЙ ОПРОС DS18B20 (Раз в 500 мс)
  if (dsFound && (currentMillis - lastDSRequestTime >= 500)) {
    cubeTemp = dsSensors.getTempC(dsCubeAddr);
    dsSensors.requestTemperatures(); // Запрос следующего замера
    lastDSRequestTime = currentMillis;
  }

  // 5. ОБНОВЛЕНИЕ ЭКРАНА (Раз в 300 мс)
  if (currentMillis - lastDisplayUpdate >= 300) {
    lastDisplayUpdate = currentMillis;

    // СТРОКА 1: Давление (мм рт.ст.) и Состояние Соленоида
    lcd.setCursor(0, 0);
    if (bmpOK) {
      float mmHg = bmp.readPressure() * 0.00750062;
      lcd.print("P:"); 
      lcd.print(mmHg, 1);
    } else {
      lcd.print("P:ERR  ");
    }

    lcd.setCursor(9, 0);
    if (isBoosting)            lcd.print(" V:BOOST");
    else if (isSolenoidActive) lcd.print(" V:HOLD ");
    else                       lcd.print(" V:OFF  ");

    // СТРОКА 2: Температура TS1 (Pt1000) и T_Куба (DS18B20)
    lcd.setCursor(0, 1);
    float ptTemp = readPt1000();
    
    // Формат: "Pt:78.4 C:82.1 "
    lcd.print("Pt:");
    if (ptTemp > -50 && ptTemp < 200) {
      lcd.print(ptTemp, 1);
    } else {
      lcd.print("ERR");
    }

    lcd.setCursor(8, 1);
    lcd.print(" C:");
    if (dsFound && cubeTemp > -50 && cubeTemp < 125) {
      lcd.print(cubeTemp, 1);
      lcd.print(" ");
    } else {
      lcd.print("ERR ");
    }
  }
}