/* 
  ПО для портативной системы нагревателя с DST 013 OLED
  Микроконтроллер: ESP32 DevKit V1
  Дисплей: DST 013 OLED 128x64 (SPI) с библиотекой U8g2
  Ограничение температуры: максимум 10°C
*/

// =================== БИБЛИОТЕКИ ===================
#include <U8g2lib.h>
#include <SPI.h>
#include <max6675.h>
#include "soc/soc_caps.h"

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

// Пин для измерения напряжения батареи
#define BATTERY_PIN 34

// Пины для OLED дисплея (SPI)
#define OLED_CS     5   // Chip Select
#define OLED_DC     2   // Data/Command
#define OLED_RST    4   // Reset

// =================== НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ ===================
#define TEMP_UPDATE_INTERVAL 1000    // Интервал опроса температуры (мс)
#define DISPLAY_UPDATE_INTERVAL 500  // Интервал обновления дисплея (мс)
#define PWM_FREQUENCY 1000          // Частота ШИМ (Гц)
#define PWM_RESOLUTION 8            // Разрешение ШИМ (бит)
#define MAX_TEMPERATURE 10.0        // МАКСИМАЛЬНАЯ температура 10°C
#define HEATER_POWER 50             // Мощность нагревателя (0-100%)
#define BUTTON_DEBOUNCE 300         // Задержка антидребезга (мс)

// =================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ И ПЕРЕМЕННЫЕ ===================
// Инициализация OLED дисплея с U8g2
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(
  U8G2_R0,  // rotation (R0 - нормально, R2 - перевернуто на 180°)
  OLED_CS,  // Chip Select
  OLED_DC,  // Data/Command
  OLED_RST  // Reset
);

MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Температурные переменные
float currentTemp = 0.0;
bool heaterEnabled = false;
bool systemOn = true;
int heaterPower = 0;

// Переменные для батареи
float batteryVoltage = 3.7;
int batteryPercentage = 83;  // ДОБАВЛЕНО: инициализация переменной

// Тайминги
unsigned long lastTempUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastButtonPress = 0;
unsigned long lastBatteryCheck = 0;  // ДОБАВЛЕНО

// =================== ФУНКЦИЯ НАСТРОЙКИ ===================
void setup() {
  Serial.begin(115200);
  Serial.println("Система нагревателя: Запуск...");
  
  // Настройка пинов
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);  // ДОБАВЛЕНО
  digitalWrite(HEATER_PIN, LOW);
  
  // Настройка ШИМ для MOSFET
  ledcAttach(HEATER_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(HEATER_PIN, 0);  // Выключить нагрев
  
  Serial.print("Максимальная температура: ");
  Serial.print(MAX_TEMPERATURE);
  Serial.println("°C");
  
  // Инициализация OLED дисплея с U8g2
  u8g2.begin();
  
  Serial.println("OLED дисплей инициализирован");
  
  // Инициализация термопары
  delay(500);
  
  // Проверка термопары
  float testTemp = thermocouple.readCelsius();
  if(isnan(testTemp)) {
    Serial.println("ПРЕДУПРЕЖДЕНИЕ: Проверьте термопару!");
  } else {
    Serial.print("Термопара OK. Температура: ");
    Serial.print(testTemp);
    Serial.println("°C");
  }
  
  delay(1000);
}

// =================== ОСНОВНОЙ ЦИКЛ ===================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. ОПРОС ТЕМПЕРАТУРЫ
  if (currentTime - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
    updateTemperature();
    lastTempUpdate = currentTime;
  }
  
  // 2. УПРАВЛЕНИЕ НАГРЕВАТЕЛЕМ
  controlHeater();
  
  // 3. ОБРАБОТКА КНОПОК
  if (currentTime - lastButtonCheck >= 200) {
    handleButtons();
    lastButtonCheck = currentTime;
  }
  
  // 4. ОБНОВЛЕНИЕ ДИСПЛЕЯ
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // 5. ОБНОВЛЕНИЕ СОСТОЯНИЯ БАТАРЕИ (ДОБАВЛЕНО)
  if (currentTime - lastBatteryCheck >= 2000) {
    updateBatteryStatus();
    lastBatteryCheck = currentTime;
  }
  
  // 6. ПРОВЕРКА БЕЗОПАСНОСТИ
  safetyCheck();
}

// =================== ФУНКЦИЯ: ОБНОВЛЕНИЕ ТЕМПЕРАТУРЫ ===================
void updateTemperature() {
  currentTemp = thermocouple.readCelsius();
  
  if (isnan(currentTemp)) {
    Serial.println("ОШИБКА чтения термопары!");
    currentTemp = 0.0;
    heaterEnabled = false;
  }
}

// =================== ФУНКЦИЯ: ОБНОВЛЕНИЕ СОСТОЯНИЯ БАТАРЕИ (ДОБАВЛЕНО) ===================
void updateBatteryStatus() {
  // Чтение напряжения батареи (ESP32: 12-битный АЦП, 0-3.3В)
  int rawValue = analogRead(BATTERY_PIN);
  batteryVoltage = rawValue * 3.3 / 4095.0 * 2; // Делитель напряжения 1:1
  
  // Преобразование в проценты (примерно для Li-ion 3.0-4.2В)
  batteryPercentage = map(constrain(batteryVoltage, 3.0, 4.2), 3.0, 4.2, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // Отладка
  Serial.print("Батарея: ");
  Serial.print(batteryVoltage);
  Serial.print("V, ");
  Serial.print(batteryPercentage);
  Serial.println("%");
}

// =================== ФУНКЦИЯ: УПРАВЛЕНИЕ НАГРЕВАТЕЛЕМ ===================
void controlHeater() {
  if (!systemOn) {
    ledcWrite(HEATER_PIN, 0);
    heaterPower = 0;
    return;
  }
  
  if (!heaterEnabled) {
    ledcWrite(HEATER_PIN, 0);
    heaterPower = 0;
    return;
  }
  
  if (currentTemp >= MAX_TEMPERATURE) {
    ledcWrite(HEATER_PIN, 0);
    heaterPower = 0;
    Serial.println("Достигнута максимальная температура 10°C!");
    return;
  }
  
  if (currentTemp < MAX_TEMPERATURE - 2) {
    int pwmValue = map(HEATER_POWER, 0, 100, 0, 255);
    ledcWrite(HEATER_PIN, pwmValue);
    heaterPower = HEATER_POWER;
  } else if (currentTemp >= MAX_TEMPERATURE - 2 && currentTemp < MAX_TEMPERATURE) {
    int reducedPower = map(currentTemp, MAX_TEMPERATURE - 2, MAX_TEMPERATURE, HEATER_POWER, 0);
    reducedPower = constrain(reducedPower, 0, HEATER_POWER);
    int pwmValue = map(reducedPower, 0, 100, 0, 255);
    ledcWrite(HEATER_PIN, pwmValue);
    heaterPower = reducedPower;
  }
}

// =================== ФУНКЦИЯ: ОБРАБОТКА КНОПОК ===================
void handleButtons() {
  if (millis() - lastButtonPress < BUTTON_DEBOUNCE) {
    return;
  }
  
  if (digitalRead(BUTTON_POWER) == LOW) {
    systemOn = !systemOn;
    if (!systemOn) {
      heaterEnabled = false;
      ledcWrite(HEATER_PIN, 0);
    }
    Serial.print("Система: ");
    Serial.println(systemOn ? "ON" : "OFF");
    lastButtonPress = millis();
  }
  
  if (digitalRead(BUTTON_UP) == LOW) {
    if (systemOn && currentTemp < MAX_TEMPERATURE) {
      heaterEnabled = !heaterEnabled;
      Serial.print("Нагрев: ");
      Serial.println(heaterEnabled ? "ON" : "OFF");
    } else if (!systemOn) {
      Serial.println("Включите систему сначала!");
    }
    lastButtonPress = millis();
  }
  
  if (digitalRead(BUTTON_DOWN) == LOW) {
    Serial.print("Т: ");
    Serial.print(currentTemp);
    Serial.print("C, P: ");
    Serial.print(heaterPower);
    Serial.println("%");
    lastButtonPress = millis();
  }
}

// =================== ФУНКЦИЯ: ОБНОВЛЕНИЕ ДИСПЛЕЯ ===================
void updateDisplay() {
  u8g2.clearBuffer();
  
  // Устанавливаем черный фон
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 128, 64);
  u8g2.setDrawColor(1);
  
  // 1. "STANDBY" - верхний центр, отступ 5px сверху
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(5, 7, "STANDBY");
  
  // ========== ТЕМПЕРАТУРЫ ==========
  
  // 2. Текущая температура (крупная, 28pt) - динамическое позиционирование
  u8g2.setFont(u8g2_font_logisoso28_tn);
  
  // Форматируем текущую температуру
  char tempDisplay[5];
  sprintf(tempDisplay, "%d", (int)currentTemp);
  int tempWidth = u8g2.getStrWidth(tempDisplay);
  
  // Рассчитываем позицию текущей температуры левее центра
  int tempX = (128 - tempWidth) / 2 - 15; // Как было: смещение на 15px левее от центра
  int tempY = 45;
  
  // Рисуем текущую температуру
  u8g2.drawStr(tempX, tempY, tempDisplay);
  
  // Кружок градуса для текущей температуры
  drawTinyDegree(tempX + tempWidth + 3, tempY - 28);
  
  // 9. Значок нагрева или снежинки справа от текущей температуры
  int symbolX = tempX + tempWidth + 15; // 15px правее температуры
  
  if (systemOn) { //heaterEnabled && 
    // Нагрев активен - показываем 3 кривые полоски (печка)
    drawHeatingSymbol(symbolX, tempY + 10);
  } else if (!systemOn) { //&& !heaterEnabled
    // Система включена, но нагрев не активен - показываем снежинку (охлаждение)
    drawSnowflakeSymbol(symbolX, tempY + 10);
  }
  // Если systemOn = false - ничего не показываем
  
  // 3. Стрелка от текущей температуры к 80
  int arrowStartX = tempX + tempWidth + 10; // Начало стрелки (10px правее температуры)
  int arrowEndX = 85; // Конец стрелки (как было: левее "80")
  int arrowY = tempY - 10; // Стрелка выше температуры
  
  // Линия стрелки
  u8g2.drawLine(arrowStartX, arrowY, arrowEndX, arrowY);
  
  // Наконечник стрелки (треугольник)
  u8g2.drawLine(arrowEndX, arrowY, arrowEndX - 3, arrowY - 2);
  u8g2.drawLine(arrowEndX, arrowY, arrowEndX - 3, arrowY + 2);
  
  // 4. Целевая температура "80" (меньшая, 18pt) - на правой стороне как было
  u8g2.setFont(u8g2_font_logisoso18_tn);
  char targetDisplay[5] = "80";
  int targetWidth = u8g2.getStrWidth(targetDisplay);
  
  // Позиция "80" как было: 95 по X, 45 по Y (но смещаем вниз для выравнивания)
  int targetX = 95; // Как было: фиксированная позиция справа
  int targetY = tempY; // Выравниваем по базовой линии с крупной температурой
  
  u8g2.drawStr(targetX, targetY, targetDisplay);
  
  // Кружок градуса для "80"
  drawTinyDegree(targetX + targetWidth + 3, targetY - 20);
  
  // ========== БАТАРЕЯ И ПРОЦЕНТЫ ==========
  
  // 5. Рисунок батареи - правый верхний угол
  drawBattery(105, 3);

  // 6. Процент заряда - с динамическим позиционированием левее батареи
  u8g2.setFont(u8g2_font_helvB08_tr);
  char batStr[5];
  sprintf(batStr, "%d%%", batteryPercentage);
  int batWidth = u8g2.getStrWidth(batStr);
  
  // Позиция процентов: левее батареи
  int batX = 105 - batWidth - 5;
  u8g2.drawStr(batX, 12, batStr);

  // 7. "CELSING" - левый нижний угол
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(5, 62, "CELSING");

  // 8. "SET" - правый верхний угол, под батареей
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(115, 22, "SET");
  
  u8g2.sendBuffer();
}

// =================== ФУНКЦИЯ: РИСОВАНИЕ ЗНАЧКА НАГРЕВА (ПЛАМЯ) ===================
void drawHeatingSymbol(int x, int y) {
  // Рисуем пламя/огонь - классический символ нагрева
  
  // Внешний контур пламени
  // Левая сторона
  u8g2.drawLine(x + 4, y, x + 2, y + 2);
  u8g2.drawLine(x + 2, y + 2, x + 1, y + 5);
  u8g2.drawLine(x + 1, y + 5, x + 2, y + 8);
  
  // Правая сторона
  u8g2.drawLine(x + 4, y, x + 6, y + 2);
  u8g2.drawLine(x + 6, y + 2, x + 7, y + 5);
  u8g2.drawLine(x + 7, y + 5, x + 6, y + 8);
  
  // Нижняя часть (языки пламени)
  u8g2.drawLine(x + 2, y + 8, x + 3, y + 6);
  u8g2.drawLine(x + 3, y + 6, x + 4, y + 8);
  u8g2.drawLine(x + 4, y + 8, x + 5, y + 6);
  u8g2.drawLine(x + 5, y + 6, x + 6, y + 8);
  
  // Внутренние языки пламени (для объема)
  u8g2.drawLine(x + 3, y + 2, x + 4, y + 4);
  u8g2.drawLine(x + 4, y + 4, x + 5, y + 2);
  
  // Верхняя точка пламени
  u8g2.drawPixel(x + 4, y - 1);
}

// =================== ФУНКЦИЯ: РИСОВАНИЕ КРАСИВОЙ СНЕЖИНКИ ===================
void drawSnowflakeSymbol(int x, int y) {
  // Красивая снежинка с 6 лучами
  int centerX = x + 4;
  int centerY = y + 4;
  
  // Основные лучи (горизонтальные и вертикальные)
  u8g2.drawLine(centerX - 4, centerY, centerX - 1, centerY); // Левый
  u8g2.drawLine(centerX + 1, centerY, centerX + 4, centerY); // Правый
  u8g2.drawLine(centerX, centerY - 4, centerX, centerY - 1); // Верхний
  u8g2.drawLine(centerX, centerY + 1, centerX, centerY + 4); // Нижний
  
  // Диагональные лучи
  u8g2.drawLine(centerX - 3, centerY - 3, centerX - 1, centerY - 1); // Левый-верхний
  u8g2.drawLine(centerX + 3, centerY + 3, centerX + 1, centerY + 1); // Правый-нижний
  u8g2.drawLine(centerX + 3, centerY - 3, centerX + 1, centerY - 1); // Правый-верхний
  u8g2.drawLine(centerX - 3, centerY + 3, centerX - 1, centerY + 1); // Левый-нижний
  
  // Короткие поперечные линии на концах лучей
  // Горизонтальные концы
  u8g2.drawLine(centerX - 4, centerY - 1, centerX - 4, centerY + 1);
  u8g2.drawLine(centerX + 4, centerY - 1, centerX + 4, centerY + 1);
  
  // Вертикальные концы
  u8g2.drawLine(centerX - 1, centerY - 4, centerX + 1, centerY - 4);
  u8g2.drawLine(centerX - 1, centerY + 4, centerX + 1, centerY + 4);
  
  // Диагональные концы
  u8g2.drawPixel(centerX - 3, centerY - 2);
  u8g2.drawPixel(centerX - 2, centerY - 3);
  u8g2.drawPixel(centerX + 3, centerY + 2);
  u8g2.drawPixel(centerX + 2, centerY + 3);
  u8g2.drawPixel(centerX + 3, centerY - 2);
  u8g2.drawPixel(centerX + 2, centerY - 3);
  u8g2.drawPixel(centerX - 3, centerY + 2);
  u8g2.drawPixel(centerX - 2, centerY + 3);
}
// =================== ФУНКЦИЯ: РИСОВАНИЕ МАЛЕНЬКОГО КРУЖКА ГРАДУСА ===================
void drawTinyDegree(int x, int y) {
  u8g2.drawPixel(x+1, y);
  u8g2.drawPixel(x+2, y);
  u8g2.drawPixel(x+3, y);
  
  // Нижняя линия
  u8g2.drawPixel(x+1, y+4);
  u8g2.drawPixel(x+2, y+4);
  u8g2.drawPixel(x+3, y+4);
  
  // Левая линия
  u8g2.drawPixel(x, y+1);
  u8g2.drawPixel(x, y+2);
  u8g2.drawPixel(x, y+3);
  
  // Правая линия
  u8g2.drawPixel(x+4, y+1);
  u8g2.drawPixel(x+4, y+2);
  u8g2.drawPixel(x+4, y+3);
  
  // Углы для более круглой формы
  u8g2.drawPixel(x+1, y+1);
  u8g2.drawPixel(x+3, y+1);
  u8g2.drawPixel(x+1, y+3);
  u8g2.drawPixel(x+3, y+3);
}

void drawBattery(int x, int y) {
  // Контур батарейки
  u8g2.drawFrame(x, y, 20, 10);
  u8g2.drawBox(x + 20, y + 3, 2, 4); // Положительный контакт
  
  // Уровень заряда в зависимости от процентов
  int fillWidth = map(batteryPercentage, 0, 100, 0, 18);
  fillWidth = constrain(fillWidth, 0, 18);
  u8g2.drawBox(x + 1, y + 1, fillWidth, 8);
}
// =================== ФУНКЦИЯ: БЕЗОПАСНОСТЬ ===================
void safetyCheck() {
  if (currentTemp > MAX_TEMPERATURE + 2) {
    heaterEnabled = false;
    ledcWrite(HEATER_PIN, 0);
    Serial.println("АВАРИЯ: Превышение температуры!");
  }
  
  static int errorCount = 0;
  if (isnan(currentTemp)) {
    errorCount++;
    if (errorCount > 3) {
      heaterEnabled = false;
      systemOn = false;
      ledcWrite(HEATER_PIN, 0);
      Serial.println("АВАРИЯ: Потеря связи с датчиком!");
    }
  } else {
    errorCount = 0;
  }
  
  // Проверка напряжения батареи
  if (batteryPercentage < 10) {
    Serial.println("ПРЕДУПРЕЖДЕНИЕ: Низкий заряд батареи!");
  }
}