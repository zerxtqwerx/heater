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

// Тайминги
unsigned long lastTempUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastButtonPress = 0;

// =================== ФУНКЦИЯ НАСТРОЙКИ ===================
void setup() {
  Serial.begin(115200);
  Serial.println("Система нагревателя: Запуск...");
  
  // Настройка пинов
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);
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
  
  // 5. ПРОВЕРКА БЕЗОПАСНОСТИ
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
  
  // Температура (крупный шрифт слева)
  u8g2.setFont(u8g2_font_logisoso32_tn);
  char currentTempStr[10];
  dtostrf(currentTemp, 4, 1, currentTempStr);
  
  // Вывод текущей температуры (сдвинуто влево для баланса)
  u8g2.drawStr(-15, 40, currentTempStr);
  
  // Знак Цельсия
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.drawStr(60, 40, "C");  // Просто "C" без двоеточия
  
  // Максимальная температура (маленький шрифт рядом)
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(85, 40, "/");
  
  char maxTempStr[5];
  dtostrf(MAX_TEMPERATURE, 2, 0, maxTempStr);
  u8g2.drawStr(95, 40, maxTempStr);
  
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(115, 40, "C");  // Просто "C" без двоеточия
  
  // Батарейка в правом верхнем углу
  drawBattery(100, 5);
  
  // Мощность нагрева под батарейкой
  u8g2.setFont(u8g2_font_5x7_tr);
  char powerStr[10];
  sprintf(powerStr, "%d%%", heaterPower);
  u8g2.drawStr(85, 13, powerStr);
  
  // Огонек или снежинка в левом нижнем углу
  if (systemOn) { //heaterEnabled 
    drawFire(5, 47);  // НОВЫЙ огонек при нагреве
  } else {
    drawSnowflake(5, 50);  // Снежинка когда выключено
  }
  
  u8g2.sendBuffer();
}

// =================== ФУНКЦИЯ: РИСОВАНИЕ НОВОГО ОГОНЬКА ===================
void drawFire(int x, int y) {
  // Три чистых языка пламени без искр
  
  // Центральный язык (самый высокий)
  u8g2.drawTriangle(x+5, y,     // Верхняя точка
                    x+2, y+10,  // Левая нижняя
                    x+8, y+10); // Правая нижняя
  
  // Левый язык (полевее и чуть ниже)
  u8g2.drawTriangle(x+2, y+2,   // Верхняя точка
                    x, y+12,    // Левая нижняя  
                    x+4, y+12); // Правая нижняя
  
  // Правый язык (поправее и чуть ниже)
  u8g2.drawTriangle(x+8, y+2,   // Верхняя точка
                    x+6, y+12,  // Левая нижняя
                    x+10, y+12);// Правая нижняя
  
  // Две линии для скругления огня снизу
  
  // Первая линия - по всей ширине огня (от x до x+10, на высоте y+12)
  u8g2.drawHLine(x+1, y+12, 9);  // 11 пикселей от x до x+10
  
  // Вторая линия - на 2 пикселя короче (от x+1 до x+9, на высоте y+13)
  u8g2.drawHLine(x+1, y+13, 9);  // 9 пикселей от x+1 до x+9
  
  // Третья линия (опционально) - еще короче для эффекта скругления
  u8g2.drawHLine(x+2, y+14, 7);  // 7 пикселей от x+2 до x+8
}

// =================== ФУНКЦИЯ: РИСОВАНИЕ СНЕЖИНКИ ===================
void drawSnowflake(int x, int y) {
  // Центр снежинки
  int centerX = x + 5;
  int centerY = y + 5;
  
  // Вертикальная и горизонтальная линии
  u8g2.drawVLine(centerX, centerY-4, 9);
  u8g2.drawHLine(centerX-4, centerY, 9);
  
  // Диагональные линии
  u8g2.drawLine(centerX-3, centerY-3, centerX+3, centerY+3);
  u8g2.drawLine(centerX+3, centerY-3, centerX-3, centerY+3);
  
  // Маленькие линии на концах
  u8g2.drawHLine(centerX-1, centerY-4, 3);
  u8g2.drawHLine(centerX-1, centerY+4, 3);
  u8g2.drawVLine(centerX-4, centerY-1, 3);
  u8g2.drawVLine(centerX+4, centerY-1, 3);
}

// =================== ФУНКЦИЯ: РИСОВАНИЕ БАТАРЕЙКИ ===================
void drawBattery(int x, int y) {
  // Контур батарейки
  u8g2.drawFrame(x, y, 20, 10);
  u8g2.drawBox(x + 20, y + 3, 2, 4);
  
  // Уровень заряда (всегда полный для простоты)
  u8g2.drawBox(x + 2, y + 2, 16, 6);
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
}