/* 
  ПО для портативной системы нагревателя
  Микроконтроллер: ESP32 DevKit V1
  Реальные компоненты: MAX6675, MOSFET, OLED 128x64
*/

// =================== БИБЛИОТЕКИ ===================
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <max6675.h>

// =================== ОПРЕДЕЛЕНИЕ ПИНОВ ===================
// Управление MOSFET (нагревателем)
#define HEATER_PIN 12
// Пины для термопары MAX6675
#define THERMO_CLK 26
#define THERMO_CS  25
#define THERMO_DO  33
// Пины для кнопок
#define BUTTON_POWER 32
#define BUTTON_UP    14
#define BUTTON_DOWN  27
// Пин для измерения напряжения батареи (через делитель)
#define BATTERY_PIN 34

// =================== НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ ===================
/*
  Параметр: TEMP_UPDATE_INTERVAL
  Назначение: Интервал опроса датчика температуры
  Диапазон: 100-5000 мс
  Единицы: миллисекунды
*/
#define TEMP_UPDATE_INTERVAL 1000

/*
  Параметр: DISPLAY_UPDATE_INTERVAL
  Назначение: Интервал обновления информации на дисплее
  Диапазон: 500-10000 мс
  Единицы: миллисекунды
*/
#define DISPLAY_UPDATE_INTERVAL 2000

/*
  Параметр: PWM_FREQUENCY
  Назначение: Частота ШИМ сигнала для управления MOSFET
  Диапазон: 1-5000 Гц
  Единицы: Герцы
*/
#define PWM_FREQUENCY 1000

/*
  Параметр: PWM_DUTY
  Назначение: Скрыжность ШИМ сигнала (50% = равномерный нагрев)
  Диапазон: 0-255
  Единицы: единицы ШИМ (0-255 соответствует 0-100%)
*/
#define PWM_DUTY 128  // 50% скважность

/*
  Параметр: BUTTON_HOLD_TIME
  Назначение: Время удержания кнопки для включения/выключения системы
  Диапазон: 1000-5000 мс
  Единицы: миллисекунды
*/
#define BUTTON_HOLD_TIME 2000

/*
  Параметр: INACTIVITY_TIMEOUT
  Назначение: Время бездействия до отключения дисплея
  Диапазон: 5000-60000 мс
  Единицы: миллисекунды
*/
#define INACTIVITY_TIMEOUT 10000

/*
  Параметр: LOW_BATTERY_LEVEL
  Назначение: Уровень заряда батареи для предупреждения
  Диапазон: 10-30 %
  Единицы: проценты
*/
#define LOW_BATTERY_LEVEL 25

/*
  Параметр: LOW_BATTERY_WARN_INTERVAL
  Назначение: Интервал повторения предупреждения о низком заряде
  Диапазон: 300000-1800000 мс (5-30 минут)
  Единицы: миллисекунды
*/
#define LOW_BATTERY_WARN_INTERVAL 600000  // 10 минут

/*
  Параметр: BATTERY_MAX_VOLTAGE
  Назначение: Максимальное напряжение батареи (4S Li-ion)
  Диапазон: 14.0-16.8 В
  Единицы: Вольты
*/
#define BATTERY_MAX_VOLTAGE 16.8

/*
  Параметр: BATTERY_MIN_VOLTAGE
  Назначение: Минимальное напряжение батареи (разряженной)
  Диапазон: 10.0-12.0 В
  Единицы: Вольты
*/
#define BATTERY_MIN_VOLTAGE 12.0

// =================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ И ПЕРЕМЕННЫЕ ===================
Adafruit_SSD1306 display(128, 64, &Wire, -1);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Температурные переменные
float currentTemp = 0.0;
float targetTemp = 200.0;

// Флаги состояния системы
bool systemOn = false;
bool heatingOn = false;
bool displayOn = true;

// Тайминги
unsigned long lastTempUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastActivity = 0;
unsigned long lastBatteryWarn = 0;

// Канал ШИМ
const int pwmChannel = 0;  // Используем канал 0 для ШИМ

// =================== ФУНКЦИЯ НАСТРОЙКИ ===================
void setup() {
  Serial.begin(115200);
  Serial.println("Система нагревателя: Инициализация...");
  
  // Настройка пинов
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  
  // Настройка ШИМ для MOSFET
  ledcSetup(pwmChannel, PWM_FREQUENCY, 8);  // Канал, частота, разрешение 8 бит
  ledcAttachPin(HEATER_PIN, pwmChannel);    // Привязка пина к каналу
  ledcWrite(pwmChannel, 0);                 // Выключить нагрев
  
  // Инициализация дисплея
  Wire.begin(21, 22);  // SDA=21, SCL=22 для ESP32
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("ОШИБКА: OLED дисплей не найден!");
    while(1);
  }
  Serial.println("OLED дисплей инициализирован");
  
  // Настройка дисплея
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Портативный нагреватель");
  display.println("Версия 1.0");
  display.display();
  
  // Инициализация термопары
  delay(500);  // Даем время на инициализацию MAX6675
  
  // Проверка термопары
  float testTemp = thermocouple.readCelsius();
  if(isnan(testTemp)) {
    Serial.println("ПРЕДУПРЕЖДЕНИЕ: Термопара не отвечает");
  } else {
    Serial.print("Термопара инициализирована. Температура: ");
    Serial.print(testTemp);
    Serial.println(" C");
  }
  
  // Инициализация таймеров
  lastActivity = millis();
}

// =================== ОСНОВНОЙ ЦИКЛ ===================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. ОПРОС ТЕМПЕРАТУРЫ
  if (currentTime - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
    currentTemp = thermocouple.readCelsius();
    
    // Проверка на ошибку чтения
    if (isnan(currentTemp)) {
      Serial.println("ОШИБКА чтения термопары!");
      currentTemp = 0.0;
    }
    
    lastTempUpdate = currentTime;
  }
  
  // 2. ОБРАБОТКА КНОПОК
  handleButtons();
  
  // 3. УПРАВЛЕНИЕ НАГРЕВОМ
  controlHeater();
  
  // 4. ОБНОВЛЕНИЕ ДИСПЛЕЯ
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // 5. УПРАВЛЕНИЕ ДИСПЛЕЕМ (таймер бездействия)
  manageDisplayTimeout(currentTime);
  
  // 6. ПРОВЕРКА БАТАРЕИ
  checkBattery();
}

// =================== ФУНКЦИЯ: ОБРАБОТКА КНОПОК ===================
void handleButtons() {
  static unsigned long powerHoldStart = 0;
  static