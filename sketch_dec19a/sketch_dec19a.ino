//arduino ide 2.3.7
//boards library: arduino esp32 boards 2.0.18-arduino.5

//libraries:
#include <U8g2lib.h> //by oliver v 2.35.30
#include <SPI.h>
#include <max6675.h> //by adafruit v 1.1.2
#include <EEPROM.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_wifi.h>

// /////////////////////// ПИНЫ ПОДКЛЮЧЕНИЯ ///////////////////////
#define HEATER_PIN 13      // Управление нагревателем (MOSFET)
#define THERMO_CLK 26      // Тактовый пин термопары MAX6675
#define THERMO_CS  25      // Пин выбора термопары
#define THERMO_DO  33      // Пин данных термопары
#define BUTTON_POWER 32    // Главная кнопка (вкл/выкл, режимы)
#define BUTTON_UP    14    // Кнопка увеличения температуры
#define BUTTON_DOWN  27    // Кнопка уменьшения температуры
#define BATTERY_PIN 34     // Измерение напряжения батареи 
#define OLED_CS     5      // Пин CS OLED дисплея
#define OLED_DC     2      // Пин DC OLED
#define OLED_RST    4      // Пин сброса OLED

// /////////////////////// НАСТРОЙКИ ТЕМПЕРАТУРЫ ///////////////////////
#define TEMP_MIN 0         // Минимальная температура, диапазон: 0-250C
#define TEMP_MAX 250       // Максимальная температура, диапазон: 0-250C
#define TEMP_STEP 1        // Шаг изменения температуры кнопками, диапазон: 1-10C
#define DEFAULT_TEMP 200   // Температура по умолчанию при первом включении, диапазон: 0-250C

// /////////////////////// ИНТЕРВАЛЫ ВРЕМЕНИ ///////////////////////
#define TEMP_UPDATE_INTERVAL 1000    // Интервал опроса датчика температуры, диапазон: 500-5000 мс
#define DISPLAY_UPDATE_INTERVAL 200  // Интервал обновления дисплея, диапазон: 100-1000 мс
#define BUTTON_DEBOUNCE 200          // Задержка для подавления дребезга кнопок, диапазон: 50-500 мс
#define BUTTON_HOLD_TIME 2000        // Время удержания кнопки для вкл/выкл системы, диапазон: 1000-5000 мс
#define FULL_MODE_TIMEOUT 15000      // Таймаут полного режима дисплея, диапазон: 5000-30000 мс
#define BATTERY_UPDATE_INTERVAL 5000 // Интервал обновления показаний батареи, диапазон: 1000-10000 мс
#define BATTERY_WARNING_INTERVAL 600000  // Интервал повторения уведомления о низком заряде, диапазон: 30000-1800000 мс
#define BATTERY_WARNING_DURATION 5000    // Длительность показа уведомления о батарее, диапазон: 2000-10000 мс

// /////////////////////// НАСТРОЙКИ НАГРЕВА ///////////////////////
#define MAX_TEMPERATURE 250.0        // Максимальная безопасная температура, диапазон: 200-300C
#define HEATER_POWER_MAX 40          // Максимальная мощность нагрева, диапазон: 20-100%
#define HEATER_POWER_MAINTAIN 8      // Мощность для поддержания температуры, диапазон: 5-20%
#define HEATER_POWER_MIN 3           // Минимальная мощность нагрева, диапазон: 1-10%
#define PWM_FREQUENCY 10             // Частота ШИМ для плавного нагрева, диапазон: 1-100 Гц
#define PWM_RESOLUTION 8             // Разрешение ШИМ (бит), диапазон: 8-12 бит
#define HEATING_UPDATE_INTERVAL 300  // Интервал обновления мощности нагрева, диапазон: 100-1000 мс

// /////////////////////// НАСТРОЙКИ БАТАРЕИ ///////////////////////
#define MIN_VOLTAGE_FOR_HEATING 3.30 // Минимальное напряжение для работы нагрева, диапазон: 3.0-3.5 В
#define BATTERY_LOW_THRESHOLD 25     // Порог низкого заряда батареи, диапазон: 15-30%
#define BATTERY_CRITICAL_THRESHOLD 10 // Порог критического заряда батареи, диапазон: 5-15%

// /////////////////////// ПАРАМЕТРЫ ФИЛЬТРАЦИИ БАТАРЕИ ///////////////////////
#define BATTERY_SAMPLES 20           // Количество сэмплов для усреднения, диапазон: 5-50
#define BATTERY_MEDIAN_FILTER_SIZE 7 // Размер медианного фильтра, диапазон: 3-15
#define BATTERY_SMOOTH_ALPHA 0.15f   // Коэффициент сглаживания, диапазон: 0.05-0.5

// /////////////////////// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ///////////////////////

// Переменные для фильтрации батареи
float filteredBatteryPercentage = 100.0;  // Отфильтрованное значение заряда
int displayedBatteryPercentage = 100;     // Отображаемое значение (с гистерезисом)

// Буферы для фильтрации АЦП
int batterySamples[BATTERY_SAMPLES];
int batterySampleIndex = 0;
long batterySampleSum = 0;

// Медианный фильтр для устранения выбросов
unsigned int medianHistory[BATTERY_MEDIAN_FILTER_SIZE];
int medianHistoryIndex = 0;

// Объекты библиотек
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI oled(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Переменные состояния системы
float currentTemp = 0.0;           // Текущая температура с фильтрацией
float desiredTemp = DEFAULT_TEMP;  // Желаемая температура, установленная пользователем
float targetTemp = DEFAULT_TEMP;   // Целевая температура для алгоритма нагрева
bool heaterEnabled = false;        // Флаг включения нагревателя
bool systemOn = false;             // Флаг включения всей системы
bool displayOn = false;            // Флаг включения дисплея
bool fullModeActive = false;       // Флаг активного полного режима отображения
unsigned long fullModeStartTime = 0; // Время начала полного режима

// Режимы работы системы нагрева
enum HeatingMode {
  MODE_IDLE,        // Ожидание, нагрев выключен
  MODE_HEATING,     // Активный нагрев до целевой температуры
  MODE_COOLING,     // Охлаждение (перегрев или выключен нагрев)
  MODE_MAINTAIN     // Поддержание температуры в узком диапазоне
};

HeatingMode currentMode = MODE_IDLE;
HeatingMode lastActiveMode = MODE_IDLE; // Последний активный режим перед поддержанием
HeatingMode maintainSymbol = MODE_COOLING; // Какой символ показывать в режиме поддержания
bool firstTimeMaintain = false;    // Флаг первого входа в режим поддержания

// Переменные для плавного управления нагревом
float heatingPower = 0.0;          // Текущая мощность нагрева в процентах
float lastHeatingPower = 0.0;      // Предыдущая мощность для плавности
unsigned long lastHeatingUpdate = 0; // Время последнего обновления нагрева
bool overshootDetected = false;    // Флаг обнаружения перегрева
float overshootAmount = 0.0;       // Величина перегрева

// Фильтр для температуры (медианный + экспоненциальное сглаживание)
float tempHistory[5] = {0, 0, 0, 0, 0};
int tempHistoryIndex = 0;

// Переменные состояния батареи
float batteryVoltage = 0.0;        // Напряжение батареи в вольтах
int batteryPercentage = 100;       // Процент заряда батареи
unsigned long lastBatteryCheck = 0; // Время последней проверки батареи
bool criticalBattery = false;      // Флаг критического заряда батареи
bool batteryLowWarningActive = false; // Флаг низкого заряда (25%)
unsigned long lastBatteryWarningTime = 0; // Время последнего уведомления
bool showingBatteryWarning = false; // Флаг показа уведомления о батарее

// Переменные состояния кнопок
unsigned long buttonPowerPressTime = 0;   // Время нажатия кнопки POWER
unsigned long buttonPowerReleaseTime = 0; // Время отпускания кнопки POWER
int buttonPowerClickCount = 0;            // Счетчик кликов кнопки POWER
bool buttonPowerPressed = false;          // Флаг нажатия кнопки POWER
bool buttonPowerLongPress = false;        // Флаг длительного нажатия
bool lastButtonState = HIGH;              // Предыдущее состояние кнопки POWER

bool lastButtonUpState = HIGH;            // Предыдущее состояние кнопки UP
bool lastButtonDownState = HIGH;          // Предыдущее состояние кнопки DOWN
unsigned long buttonUpPressTime = 0;      // Время нажатия кнопки UP
unsigned long buttonDownPressTime = 0;    // Время нажатия кнопки DOWN
bool buttonUpPressed = false;             // Флаг нажатия кнопки UP
bool buttonDownPressed = false;           // Флаг нажатия кнопки DOWN

// Таймеры для периодических обновлений
unsigned long lastTempUpdate = 0;        // Время последнего обновления температуры
unsigned long lastDisplayUpdate = 0;     // Время последнего обновления дисплея
unsigned long lastActivity = 0;          // Время последней активности пользователя

// /////////////////////// ФУНКЦИИ УПРАВЛЕНИЯ ДИСПЛЕЕМ ///////////////////////

/**
 * Аппаратный сброс дисплея для устранения артефактов
 * Выполняется перед инициализацией библиотеки
 */
void resetDisplay() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  delay(1);
  digitalWrite(OLED_RST, LOW);
  delay(10);
  digitalWrite(OLED_RST, HIGH);
  delay(100); // Ждем стабилизации после сброса
}

/**
 * Быстрая инициализация дисплея с низкоуровневыми командами
 * Позволяет избежать появления полосы при включении
 */
void initDisplay() {
  oled.begin();
  
  // Низкоуровневые команды инициализации SSD1306 (чтобы убрать полосу)
  sendCommand(0xAE);  // Display OFF - выключаем перед настройкой
  sendCommand(0xD5);  // Set Display Clock Divide Ratio/Oscillator Frequency
  sendCommand(0x80);  // Рекомендуемое значение
  sendCommand(0xA8);  // Set Multiplex Ratio
  sendCommand(0x3F);  // 1/64 duty (для 64px высоты)
  sendCommand(0xD3);  // Set Display Offset
  sendCommand(0x00);  // No offset
  sendCommand(0x40);  // Set Start Line to 0
  sendCommand(0x8D);  // Charge Pump Setting
  sendCommand(0x14);  // Enable charge pump
  sendCommand(0x20);  // Set Memory Addressing Mode
  sendCommand(0x00);  // Horizontal addressing mode
  sendCommand(0xA0);  // Segment re-map 0
  sendCommand(0xC0);  // COM Output Scan Direction 0
  sendCommand(0xDA);  // Set COM Pins Hardware Configuration
  sendCommand(0x12);  // Alternative COM pin configuration
  sendCommand(0x81);  // Set Contrast Control
  sendCommand(0x00);  // Минимальный контраст при инициализации
  sendCommand(0xD9);  // Set Pre-charge Period
  sendCommand(0x22);  // По даташиту
  sendCommand(0xDB);  // Set VCOMH Deselect Level
  sendCommand(0x20);  // 0.77 x VCC
  sendCommand(0xA6);  // Set Normal Display (не инверсный)
  sendCommand(0x2E);  // Deactivate Scroll
}

/**
 * Включение дисплея с нормальными настройками контраста
 * Вызывается после инициализации
 */
void turnOnDisplay() {
  sendCommand(0xAF);  // Display ON
  sendCommand(0x81);  // Set Contrast Control
  sendCommand(0x7F);  // Средняя контрастность
}

/**
 * Очистка всего дисплея через низкоуровневые команды
 * Эффективнее чем clearBuffer() + sendBuffer()
 */
void clearDisplay() {
  sendCommand(0x21);  // Set column address
  sendCommand(0x00);  // Start column 0
  sendCommand(0x7F);  // End column 127
  sendCommand(0x22);  // Set page address
  sendCommand(0x00);  // Start page 0
  sendCommand(0x07);  // End page 7
  
  oled.setDrawColor(0); // Черный цвет (выключить пиксели)
  oled.drawBox(0, 0, 130, 64);  // 130 чтобы покрыть возможную полосу
  oled.sendBuffer();
}

/**
 * Отправка низкоуровневой команды на дисплей по SPI
 * @param cmd - команда для отправки
 */
void sendCommand(uint8_t cmd) {
  digitalWrite(OLED_DC, LOW);    // Командный режим (DC=0)
  digitalWrite(OLED_CS, LOW);    // Выбор чипа дисплея
  SPI.transfer(cmd);             // Отправка команды
  digitalWrite(OLED_CS, HIGH);   // Отмена выбора чипа
  delayMicroseconds(10);         // Короткая пауза между командами
}

// /////////////////////// ОТКЛЮЧЕНИЕ Wi-Fi И Bluetooth ///////////////////////

/**
 * Полное отключение Wi-Fi и Bluetooth для экономии энергии
 * и предотвращения помех при работе с термопарой по SPI
 */
void disableWiFiBluetooth() {
  // Останавливаем Wi-Fi если был запущен
  if(WiFi.status() != WL_DISCONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  
  // Выключаем Wi-Fi на аппаратном уровне
  esp_wifi_stop();
  esp_wifi_deinit();
  
  // Выключаем Bluetooth
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  
  // Устанавливаем режим Wi-Fi OFF для надежности
  WiFi.mode(WIFI_OFF);
  setCpuFrequencyMhz(80); // Понижаем частоту CPU для экономии энергии
}

// /////////////////////// ФУНКЦИИ ФИЛЬТРАЦИИ БАТАРЕИ ///////////////////////

/**
 * Квадратичная формула расчета процента заряда из RAW значения АЦП
 * Калибровочная формула из скрипта калибровки
 * @param rawADC - сырое значение АЦП (0-4095 для 12-битного)
 * @return процент заряда от 0 до 100
 */
int calculateBatteryPercentageQuadratic(unsigned int rawADC) {
  // Масштабируем 12-битное значение к 10-битному эквиваленту
  float v = (float)rawADC / 4.0f;
  
  // Квадратичная формула калибровки
  int p = round(((v - 561.982f) * 0.000629f) * (v - 680.0f));
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

/**
 * Усиленное считывание АЦП с предварительной фильтрацией
 * Использует медиану из 3 измерений + скользящее среднее
 * @return отфильтрованное RAW значение АЦП
 */
unsigned int readBatteryADC() {
  // Убираем старый сэмпл из буфера скользящего среднего
  batterySampleSum -= batterySamples[batterySampleIndex];
  
  // Считываем 3 измерения для медианной фильтрации
  int measurements[3];
  for (int i = 0; i < 3; i++) {
    measurements[i] = analogRead(BATTERY_PIN);
    delayMicroseconds(50); // Пауза между измерениями
  }
  
  // Сортируем и берем медиану из 3 измерений
  int currentSample;
  // Быстрая сортировка для 3 элементов
  if (measurements[0] > measurements[1]) {
    if (measurements[1] > measurements[2]) {
      currentSample = measurements[1];
    } else if (measurements[0] > measurements[2]) {
      currentSample = measurements[2];
    } else {
      currentSample = measurements[0];
    }
  } else {
    if (measurements[0] > measurements[2]) {
      currentSample = measurements[0];
    } else if (measurements[1] > measurements[2]) {
      currentSample = measurements[2];
    } else {
      currentSample = measurements[1];
    }
  }
  
  // Добавляем медианное значение в кольцевой буфер
  batterySamples[batterySampleIndex] = currentSample;
  batterySampleSum += currentSample;
  batterySampleIndex = (batterySampleIndex + 1) % BATTERY_SAMPLES;
  
  // Возвращаем среднее из буфера
  return batterySampleSum / BATTERY_SAMPLES;
}

/**
 * Применение медианного фильтра для устранения выбросов
 * @param rawADC - входное значение АЦП
 * @return медианное значение из истории
 */
unsigned int applyMedianFilter(unsigned int rawADC) {
  // Добавляем новое значение в историю
  medianHistory[medianHistoryIndex] = rawADC;
  medianHistoryIndex = (medianHistoryIndex + 1) % BATTERY_MEDIAN_FILTER_SIZE;
  
  // Копируем историю для сортировки
  unsigned int sorted[BATTERY_MEDIAN_FILTER_SIZE];
  for (int i = 0; i < BATTERY_MEDIAN_FILTER_SIZE; i++) {
    sorted[i] = medianHistory[i];
  }
  
  // Сортировка пузырьком (медленно, но для 7 элементов приемлемо)
  for (int i = 0; i < BATTERY_MEDIAN_FILTER_SIZE - 1; i++) {
    for (int j = i + 1; j < BATTERY_MEDIAN_FILTER_SIZE; j++) {
      if (sorted[j] < sorted[i]) {
        unsigned int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  
  // Возвращаем медиану (средний элемент отсортированного массива)
  return sorted[BATTERY_MEDIAN_FILTER_SIZE / 2];
}

/**
 * Основная функция обновления статуса батареи с многоуровневой фильтрацией
 * Вызывается каждые 5 секунд (BATTERY_UPDATE_INTERVAL)
 */
void updateBatteryStatus() {
  // 1. Считываем АЦП с предварительной фильтрацией (медиана из 3 + скользящее среднее)
  unsigned int rawADC = readBatteryADC();
  
  // 2. Применяем медианный фильтр для устранения выбросов
  rawADC = applyMedianFilter(rawADC);
  
  // 3. Вычисляем процент по калибровочной квадратичной формуле
  int currentPercentage = calculateBatteryPercentageQuadratic(rawADC);
  
  // 4. Экспоненциальное сглаживание для плавности
  filteredBatteryPercentage = filteredBatteryPercentage * (1.0 - BATTERY_SMOOTH_ALPHA) + 
                             currentPercentage * BATTERY_SMOOTH_ALPHA;
  
  // 5. Гистерезис для отображения - меняем только если разница > 1%
  // Предотвращает "дребезг" процента на дисплее
  int filteredInt = (int)round(filteredBatteryPercentage);
  if (abs(filteredInt - displayedBatteryPercentage) > 1) {
    displayedBatteryPercentage = filteredInt;
  }
  
  // 6. Обновляем глобальные переменные
  batteryPercentage = displayedBatteryPercentage;
  batteryVoltage = rawADC / 992.7f; // Конверсия RAW в вольты (3.3V ref, 12-bit)
  
  // 7. Обновляем флаги состояния батареи
  criticalBattery = (batteryPercentage <= BATTERY_CRITICAL_THRESHOLD);
  batteryLowWarningActive = (batteryPercentage <= BATTERY_LOW_THRESHOLD);
}

// /////////////////////// УПРАВЛЕНИЕ РЕЖИМАМИ ДИСПЛЕЯ ///////////////////////

/**
 * Активация полного режима отображения на 15 секунд
 * В полном режиме показывается вся информация: температура, целевая температура,
 * заряд батареи, режим работы и логотип
 */
void activateFullMode() {
  fullModeActive = true;
  fullModeStartTime = millis();
  displayOn = true;
}

/**
 * Управление режимами отображения в зависимости от состояния системы
 * и активности пользователя
 */
void updateDisplayMode() {
  unsigned long currentTime = millis();
  
  // Если система выключена - дисплей должен быть выключен
  if (!systemOn) {
    if (displayOn) {
      displayOn = false;
      oled.clearBuffer();
      oled.sendBuffer();
    }
    return;
  }
  
  // Если система включена, но дисплей выключен - включаем упрощенный режим
  if (!displayOn) {
    displayOn = true;
  }
  
  // Проверяем таймаут полного режима (15 секунд)
  if (fullModeActive && (currentTime - fullModeStartTime >= FULL_MODE_TIMEOUT)) {
    fullModeActive = false; // Возвращаемся в упрощенный режим
  }
}

// /////////////////////// ОБНОВЛЕНИЕ ДИСПЛЕЯ ///////////////////////

/**
 * Основная функция отрисовки интерфейса на OLED дисплее
 * Имеет два режима: полный (fullModeActive) и упрощенный
 */
void updateDisplay() {
  if (!displayOn) return;
  
  oled.clearBuffer();
  oled.setDrawColor(1); // Белый цвет для рисования
  
  if (fullModeActive) {
    // ///////// ПОЛНЫЙ РЕЖИМ ОТОБРАЖЕНИЯ /////////
    // Показывает полную информацию о состоянии системы
    
    // Верхняя строка: статус системы
    oled.setFont(u8g2_font_micro_tr);
    if (!systemOn) {
      oled.drawStr(5, 7, "OFF");
    } else if (!heaterEnabled) {
      oled.drawStr(5, 7, "STANDBY");
    } else {
      // Показываем текущий режим работы нагрева
      switch (currentMode) {
        case MODE_HEATING: oled.drawStr(5, 7, "HEATING"); break;
        case MODE_COOLING: oled.drawStr(5, 7, "COOLING"); break;
        case MODE_MAINTAIN: oled.drawStr(5, 7, "MAINTAIN"); break;
        default: break;
      }
    }
    
    // Основная температура большим шрифтом
    oled.setFont(u8g2_font_logisoso28_tn);
    char tempDisplay[6];
    sprintf(tempDisplay, "%d", (int)currentTemp);
    int tempWidth = oled.getStrWidth(tempDisplay);
    int tempX = (128 - tempWidth) / 2 - 30; // Смещение влево для стрелки
    int tempY = 45;
    oled.drawStr(tempX, tempY, tempDisplay);
    
    // Символ градуса Цельсия
    drawDegree(tempX + tempWidth + 3, tempY - 28);
    
    // Символ режима работы (нагрев/охлаждение)
    int symbolX = tempX + tempWidth + 15;
    if (systemOn) {
      if (heaterEnabled) {
        if (currentMode == MODE_MAINTAIN) {
          // В режиме поддержания используем сохраненный символ
          if (maintainSymbol == MODE_HEATING) {
            drawHeatingSymbol(symbolX, tempY + 10);
          } else {
            drawCoolingSymbol(symbolX, tempY + 10);
          }
        } else {
          // В обычном режиме показываем текущий символ
          switch (currentMode) {
            case MODE_HEATING: drawHeatingSymbol(symbolX, tempY + 10); break;
            case MODE_COOLING: drawCoolingSymbol(symbolX, tempY + 10); break;
            default: drawCoolingSymbol(symbolX, tempY + 10); break;
          }
        }
      } else {
        drawCoolingSymbol(symbolX, tempY + 10);
      }
    }
    
    // Стрелка от текущей температуры к целевой
    int arrowStartX = tempX + tempWidth + 10;
    int arrowEndX = 80;
    int arrowY = tempY - 10;
    oled.drawLine(arrowStartX, arrowY, arrowEndX, arrowY); // Горизонтальная линия
    oled.drawLine(arrowEndX, arrowY, arrowEndX - 3, arrowY - 2); // Наконечник вверх
    oled.drawLine(arrowEndX, arrowY, arrowEndX - 3, arrowY + 2); // Наконечник вниз
    
    // Целевая температура (установленная пользователем)
    oled.setFont(u8g2_font_logisoso18_tn);
    char targetDisplay[5];
    sprintf(targetDisplay, "%d", (int)desiredTemp);
    int targetWidth = oled.getStrWidth(targetDisplay);
    int targetX = 85;
    int targetY = tempY;
    oled.drawStr(targetX, targetY, targetDisplay);
    drawDegree(targetX + targetWidth + 3, targetY - 20);
    
    // Индикатор батареи в правом верхнем углу
    drawBatteryIndicator(105, 3, batteryPercentage);
    
    // Процент заряда батареи текстом
    oled.setFont(u8g2_font_helvB08_tr);
    char batStr[8];
    sprintf(batStr, "%d%%", batteryPercentage);
    int batWidth = oled.getStrWidth(batStr);
    int batX = 105 - batWidth - 5;
    oled.drawStr(batX, 12, batStr);
    
    // Логотип и подписи
    oled.setFont(u8g2_font_micro_tr);
    oled.drawStr(5, 62, "CELSING"); // Логотип в левом нижнем углу
    oled.drawStr(115, 22, "SET");   // Подпись "SET" у целевой температуры
    
  } else {
    // ///////// УПРОЩЕННЫЙ РЕЖИМ ОТОБРАЖЕНИЯ /////////
    // Показывает только текущую температуру большим шрифтом
    // Используется для экономии энергии и минималистичного отображения
    
    oled.setFont(u8g2_font_logisoso38_tn);
    char tempStr[6];
    sprintf(tempStr, "%d", (int)currentTemp);
    int tempWidth = oled.getStrWidth(tempStr);
    int tempX = (128 - tempWidth) / 2; // Центрирование
    int tempY = 45;
    oled.drawStr(tempX, tempY, tempStr);
    
    // Символ градуса Цельсия меньшим шрифтом
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(tempX + tempWidth + 5, 30, "o");
    oled.drawStr(tempX + tempWidth + 11, 30, "C");
  }
  
  oled.sendBuffer(); // Отправляем буфер на дисплей
}

// /////////////////////// ОБРАБОТКА КНОПКИ POWER ///////////////////////

/**
 * Обработка кнопки POWER с поддержкой:
 * - Короткого нажатия (активация полного режима)
 * - Двойного клика (включение/выключение нагрева)
 * - Длительного удержания 2 сек (включение/выключение системы)
 */
void handlePowerButton() {
  unsigned long currentTime = millis();
  bool buttonState = digitalRead(BUTTON_POWER);
  
  static unsigned long lastClickTime = 0; // Время последнего клика
  static int clickCount = 0;              // Счетчик кликов для двойного клика
  
  // Обработка нажатия кнопки (передний фронт)
  if (buttonState == LOW && lastButtonState == HIGH) {
    if (currentTime - buttonPowerPressTime > 50) { // Антидребезг
      buttonPowerPressed = true;
      buttonPowerPressTime = currentTime;
      lastActivity = currentTime; // Сброс таймера бездействия
    }
  }
  
  // Обработка отпускания кнопки (задний фронт)
  if (buttonState == HIGH && lastButtonState == LOW) {
    if (buttonPowerPressed) {
      buttonPowerPressed = false;
      buttonPowerReleaseTime = currentTime;
      lastActivity = currentTime;
      
      unsigned long pressDuration = currentTime - buttonPowerPressTime;
      
      // ///////// ДЛИТЕЛЬНОЕ НАЖАТИЕ (2 секунды) /////////
      // Включение/выключение всей системы
      if (pressDuration >= BUTTON_HOLD_TIME) {
        systemOn = !systemOn;
        lastActivity = currentTime;
        clickCount = 0; // Сбрасываем счетчик кликов
        
        if (systemOn) {
          // Система включена
          activateFullMode(); // Показываем полный режим на 15 секунд
          heaterEnabled = false; // Нагрев выключен при старте
          currentMode = MODE_IDLE;
          overshootDetected = false;
          ledcWrite(0, 0); // Выключаем ШИМ
        } else {
          // Система выключена
          heaterEnabled = false;
          ledcWrite(0, 0);
          saveTemperatureToEEPROM(); // Сохраняем температуру в память
          fullModeActive = false;
          displayOn = false; // Выключаем дисплей
          oled.clearBuffer();
          oled.sendBuffer();
        }
        
        delay(300); // Задержка для предотвращения повторного срабатывания
        return;
      }
      
      // ///////// КОРОТКОЕ НАЖАТИЕ /////////
      else {
        // Логика подсчета кликов для двойного клика
        if (currentTime - lastClickTime < 500) { // 500ms окно для двойного клика
          clickCount++;
        } else {
          clickCount = 1;
        }
        lastClickTime = currentTime;
        
        // Обработка в зависимости от количества кликов
        if (clickCount == 1) {
          // Одиночный клик - активация полного режима на 15 секунд
          if (!systemOn) return; // Если система выключена, игнорируем
          
          activateFullMode();
        }
        else if (clickCount == 2) {
          // Двойной клик - включение/выключение нагрева
          clickCount = 0; // Сбрасываем счетчик
          
          if (systemOn && !criticalBattery) {
            // Проверка возможности включения нагрева
            if (!heaterEnabled && (batteryPercentage <= 0 || batteryVoltage < MIN_VOLTAGE_FOR_HEATING)) {
              return; // Недостаточное напряжение для нагрева
            }
            
            // Переключение состояния нагрева
            heaterEnabled = !heaterEnabled;
            overshootDetected = false;
            firstTimeMaintain = false;
            lastActivity = currentTime;
            activateFullMode(); // Показываем полный режим
            
            if (heaterEnabled) {
              // Нагрев включен
              targetTemp = desiredTemp; // Устанавливаем целевую температуру
              heatingPower = HEATER_POWER_MIN; // Начинаем с минимальной мощности
              lastHeatingPower = HEATER_POWER_MIN;
            } else {
              // Нагрев выключен
              ledcWrite(0, 0); // Выключаем ШИМ
              heatingPower = 0;
              lastHeatingPower = 0;
              currentMode = MODE_IDLE;
              lastActiveMode = MODE_IDLE;
              maintainSymbol = MODE_COOLING;
            }
          }
        }
      }
    }
  }
  
  lastButtonState = buttonState;
  
  // Сброс счетчика кликов через 1 секунду бездействия
  if (clickCount > 0 && (currentTime - lastClickTime > 1000)) {
    clickCount = 0;
  }
}

// /////////////////////// НАСТРОЙКА СИСТЕМЫ ///////////////////////

/**
 * Функция setup() - выполняется один раз при включении
 * Инициализирует все компоненты системы
 */
void setup() {
  Serial.begin(115200);
  
  // 1. ОТКЛЮЧЕНИЕ Wi-Fi И Bluetooth
  // Делается в первую очередь для предотвращения помех
  disableWiFiBluetooth();
  
  // 2. НАСТРОЙКА ПИНОВ
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  digitalWrite(HEATER_PIN, LOW); // Гарантированно выключаем нагрев
  
  // 3. НАСТРОЙКА ШИМ ДЛЯ НАГРЕВАТЕЛЯ
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, 0);
  ledcWrite(0, 0); // Начальное состояние - выключено
  
  // 4. ИНИЦИАЛИЗАЦИЯ ФИЛЬТРА БАТАРЕИ
  // Заполняем буфер начальными значениями
  for (int i = 0; i < BATTERY_SAMPLES; i++) {
    int sample = analogRead(BATTERY_PIN);
    batterySamples[i] = sample;
    batterySampleSum += sample;
    delayMicroseconds(100);
  }
  
  // Инициализируем медианный фильтр средним значением
  for (int i = 0; i < BATTERY_MEDIAN_FILTER_SIZE; i++) {
    medianHistory[i] = batterySampleSum / BATTERY_SAMPLES;
  }
  
  // Расчет начального процента заряда
  unsigned int initRaw = analogRead(BATTERY_PIN);
  filteredBatteryPercentage = calculateBatteryPercentageQuadratic(initRaw);
  displayedBatteryPercentage = (int)round(filteredBatteryPercentage);
  batteryPercentage = displayedBatteryPercentage;
  
  // 5. ИНИЦИАЛИЗАЦИЯ ДИСПЛЕЯ
  resetDisplay();    // Аппаратный сброс
  initDisplay();     // Низкоуровневая инициализация
  turnOnDisplay();   // Включение с нормальным контрастом
  clearDisplay();    // Очистка от возможных артефактов
  displayOn = false; // Дисплей выключен при старте
  
  // 6. ЗАГРУЗКА НАСТРОЕК ИЗ EEPROM
  EEPROM.begin(128);
  loadSavedTemperature();
  targetTemp = desiredTemp; // Устанавливаем целевую температуру
  
  // 7. ПРОВЕРКА ТЕРМОПАРЫ
  delay(500); // Даем время на инициализацию MAX6675
  float testTemp = thermocouple.readCelsius();
  if(isnan(testTemp) || testTemp < -50 || testTemp > 1000) {
    // Ошибка датчика температуры
    displayErrorMessage("SENSOR ERROR");
  } else {
    // Датчик работает, инициализируем фильтр температуры
    currentTemp = testTemp;
    for (int i = 0; i < 5; i++) {
      tempHistory[i] = currentTemp;
    }
  }
  
  // 8. ПЕРВОЕ ОБНОВЛЕНИЕ БАТАРЕИ
  updateBatteryStatus();
  lastActivity = millis(); // Сброс таймера активности
}

// /////////////////////// ГЛАВНЫЙ ЦИКЛ ///////////////////////

/**
 * Функция loop() - выполняется постоянно в цикле
 * Управляет всеми процессами системы
 */
void loop() {
  unsigned long currentTime = millis();
  
  // 1. УПРАВЛЕНИЕ РЕЖИМАМИ ДИСПЛЕЯ
  updateDisplayMode();
  
  // 2. ОБРАБОТКА КНОПКИ POWER
  handlePowerButton();
  
  // 3. ОБРАБОТКА КНОПОК ТЕМПЕРАТУРЫ (только если система включена)
  if (systemOn) {
    handleTemperatureButtons();
  }
  
  // 4. ОБНОВЛЕНИЕ ТЕМПЕРАТУРЫ (каждую секунду)
  if (currentTime - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
    updateTemperature();
    
    // ///////// ЛОГИКА УПРАВЛЕНИЯ РЕЖИМАМИ НАГРЕВА /////////
    if (heaterEnabled && !criticalBattery) {
      float tempDiff = targetTemp - currentTemp;
      float absTempDiff = abs(tempDiff);
      
      // Адаптивные пороги в зависимости от расстояния до цели
      float adaptiveOvershootThreshold = max(3.0f, absTempDiff * 0.05f);
      float adaptiveMaintainThreshold = max(1.5f, absTempDiff * 0.02f);
      
      // Обнаружение перегрева
      if (currentTemp > targetTemp + adaptiveOvershootThreshold) {
        overshootDetected = true;
        overshootAmount = currentTemp - targetTemp;
        currentMode = MODE_COOLING;
        lastActiveMode = MODE_COOLING;
        maintainSymbol = MODE_COOLING;
        firstTimeMaintain = false;
      }
      // Выход из состояния перегрева
      else if (overshootDetected) {
        if (currentTemp <= targetTemp + 0.5f) {
          overshootDetected = false;
          currentMode = MODE_MAINTAIN;
          if (!firstTimeMaintain) {
            firstTimeMaintain = true;
          }
        } else {
          currentMode = MODE_COOLING;
          lastActiveMode = MODE_COOLING;
          maintainSymbol = MODE_COOLING;
          firstTimeMaintain = false;
        }
      }
      // Нормальная работа (нагрев или поддержание)
      else {
        if (absTempDiff <= adaptiveMaintainThreshold) {
          // Достигли целевой температуры - переходим в режим поддержания
          currentMode = MODE_MAINTAIN;
          if (lastActiveMode == MODE_HEATING) {
            maintainSymbol = MODE_HEATING;
          } else {
            maintainSymbol = MODE_COOLING;
          }
          
          if (!firstTimeMaintain) {
            firstTimeMaintain = true;
          }
        }
        else if (tempDiff > 0) {
          // Температура ниже целевой - режим нагрева
          currentMode = MODE_HEATING;
          lastActiveMode = MODE_HEATING;
          firstTimeMaintain = false;
        }
        else {
          // Температура выше целевой (но не перегрев) - режим охлаждения
          currentMode = MODE_COOLING;
          lastActiveMode = MODE_COOLING;
          maintainSymbol = MODE_COOLING;
          firstTimeMaintain = false;
        }
      }
    } else {
      // Нагрев выключен или критический заряд батареи
      currentMode = MODE_COOLING;
      lastActiveMode = MODE_COOLING;
      maintainSymbol = MODE_COOLING;
      overshootDetected = false;
      firstTimeMaintain = false;
    }
    
    lastTempUpdate = currentTime;
  }
  
  // 5. УПРАВЛЕНИЕ НАГРЕВАТЕЛЕМ
  if (systemOn && heaterEnabled && !criticalBattery) {
    controlHeater();
  } else {
    // Выключаем нагрев если система выключена или критический заряд
    ledcWrite(0, 0);
    heatingPower = 0;
    lastHeatingPower = 0;
  }
  
  // 6. ОБНОВЛЕНИЕ БАТАРЕИ (каждые 5 секунд)
  if (currentTime - lastBatteryCheck >= BATTERY_UPDATE_INTERVAL) {
    updateBatteryStatus();
    lastBatteryCheck = currentTime;
  }
  
  // 7. УПРАВЛЕНИЕ УВЕДОМЛЕНИЯМИ О НИЗКОМ ЗАРЯДЕ
  if (systemOn && batteryLowWarningActive && batteryPercentage > 0) {
    if (currentTime - lastBatteryWarningTime >= BATTERY_WARNING_INTERVAL) {
      showingBatteryWarning = true;
      displayOn = true;
      lastActivity = currentTime;
      lastBatteryWarningTime = currentTime;
    }
    
    // Выключаем уведомление через 5 секунд
    if (showingBatteryWarning && (currentTime - lastBatteryWarningTime >= BATTERY_WARNING_DURATION)) {
      showingBatteryWarning = false;
    }
  } else {
    showingBatteryWarning = false;
  }
  
  // 8. ОБНОВЛЕНИЕ ДИСПЛЕЯ (кроме показа уведомления о батарее)
  if (displayOn && systemOn && !showingBatteryWarning) {
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      updateDisplay();
      lastDisplayUpdate = currentTime;
    }
  }
  
  // 9. ОТОБРАЖЕНИЕ УВЕДОМЛЕНИЯ О БАТАРЕЕ
  if (showingBatteryWarning && displayOn) {
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      displayBatteryWarning();
      lastDisplayUpdate = currentTime;
    }
  }
  
  // 10. ПРОВЕРКИ БЕЗОПАСНОСТИ
  safetyCheck();
}

// /////////////////////// ОТОБРАЖЕНИЕ УВЕДОМЛЕНИЯ О БАТАРЕЕ ///////////////////////

/**
 * Показывает уведомление о низком заряде батареи
 * Отображается поверх основного интерфейса на 5 секунд
 */
void displayBatteryWarning() {
  oled.clearBuffer();
  oled.setDrawColor(1);
  
  // Рисуем крупную иконку батареи
  int batteryX = 34;
  int batteryY = 12;
  int batteryWidth = 60;
  int batteryHeight = 40;
  
  // Контур батарейки
  oled.drawFrame(batteryX, batteryY, batteryWidth, batteryHeight);
  oled.drawBox(batteryX + batteryWidth, batteryY + 15, 6, 10); // Полюс батареи
  
  // Заливка (4 сегмента по 25%)
  int fillSegments = batteryPercentage / 25;
  if (fillSegments > 0) {
    int segmentWidth = (batteryWidth - 4) / 4;
    for (int i = 0; i < fillSegments; i++) {
      oled.drawBox(batteryX + 2 + i * segmentWidth, batteryY + 2, 
                   segmentWidth - 2, batteryHeight - 4);
    }
  }
  
  // Восклицательный знак в центре
  oled.setFont(u8g2_font_helvB24_tr);
  oled.drawStr(batteryX + batteryWidth/2 - 6, batteryY + batteryHeight/2 + 10, "!");
  
  oled.sendBuffer();
}

// /////////////////////// ОБРАБОТКА КНОПОК ТЕМПЕРАТУРЫ ///////////////////////

/**
 * Обработка кнопок UP и DOWN для изменения целевой температуры
 * Поддерживает:
 * - Короткое нажатие: изменение на 1C
 * - Удержание: плавное изменение каждые 100мс
 */
void handleTemperatureButtons() {
  unsigned long currentTime = millis();
  static unsigned long lastButtonUpTime = 0;
  static unsigned long lastButtonDownTime = 0;
  static bool buttonUpWasPressed = false;
  static bool buttonDownWasPressed = false;
  
  bool buttonUpState = digitalRead(BUTTON_UP);
  bool buttonDownState = digitalRead(BUTTON_DOWN);
  
  // ///////// ОБРАБОТКА КНОПКИ UP /////////
  if (buttonUpState == LOW && lastButtonUpState == HIGH) {
    // Нажатие кнопки UP
    if (currentTime - lastButtonUpTime > BUTTON_DEBOUNCE) {
      buttonUpWasPressed = true;
      buttonUpPressTime = currentTime;
      lastButtonUpTime = currentTime;
      lastActivity = currentTime;
      activateFullMode(); // Активируем полный режим при изменении температуры
    }
  }
  
  if (buttonUpState == HIGH && lastButtonUpState == LOW) {
    // Отпускание кнопки UP
    if (buttonUpWasPressed) {
      buttonUpWasPressed = false;
      unsigned long pressDuration = currentTime - buttonUpPressTime;
      
      // Короткое нажатие (<500ms) - изменение на 1C
      if (pressDuration < 500) {
        desiredTemp += TEMP_STEP;
        if (desiredTemp > TEMP_MAX) desiredTemp = TEMP_MAX;
        
        if (heaterEnabled) {
          targetTemp = desiredTemp; // Обновляем цель нагрева
          overshootDetected = false; // Сбрасываем флаг перегрева
        }
        
        saveTemperatureToEEPROM(); // Сохраняем в EEPROM
      }
      // Длительное нажатие (отпустили после >500ms)
      else {
        // Ничего не делаем при отпускании после длительного удержания
        // Изменения уже происходили во время удержания
      }
    }
  }
  
  // ///////// ОБРАБОТКА КНОПКИ DOWN /////////
  if (buttonDownState == LOW && lastButtonDownState == HIGH) {
    // Нажатие кнопки DOWN
    if (currentTime - lastButtonDownTime > BUTTON_DEBOUNCE) {
      buttonDownWasPressed = true;
      buttonDownPressTime = currentTime;
      lastButtonDownTime = currentTime;
      lastActivity = currentTime;
      activateFullMode(); // Активируем полный режим
    }
  }
  
  if (buttonDownState == HIGH && lastButtonDownState == LOW) {
    // Отпускание кнопки DOWN
    if (buttonDownWasPressed) {
      buttonDownWasPressed = false;
      unsigned long pressDuration = currentTime - buttonDownPressTime;
      
      // Короткое нажатие (<500ms) - изменение на -1C
      if (pressDuration < 500) {
        desiredTemp -= TEMP_STEP;
        if (desiredTemp < TEMP_MIN) desiredTemp = TEMP_MIN;
        
        if (heaterEnabled) {
          targetTemp = desiredTemp;
          overshootDetected = false;
        }
        
        saveTemperatureToEEPROM();
      }
    }
  }
  
  // ///////// ОБРАБОТКА УДЕРЖАНИЯ КНОПКИ UP /////////
  if (buttonUpState == LOW && buttonUpWasPressed) {
    unsigned long pressDuration = currentTime - buttonUpPressTime;
    
    // После 500ms начинаем плавное изменение
    if (pressDuration > 500) {
      static unsigned long lastHoldChange = 0;
      if (currentTime - lastHoldChange > 100) { // Каждые 100ms
        lastHoldChange = currentTime;
        desiredTemp += TEMP_STEP;
        if (desiredTemp > TEMP_MAX) desiredTemp = TEMP_MAX;
        
        if (heaterEnabled) {
          targetTemp = desiredTemp;
          overshootDetected = false;
        }
        
        saveTemperatureToEEPROM();
      }
    }
  }
  
  // ///////// ОБРАБОТКА УДЕРЖАНИЯ КНОПКИ DOWN /////////
  if (buttonDownState == LOW && buttonDownWasPressed) {
    unsigned long pressDuration = currentTime - buttonDownPressTime;
    
    // После 500ms начинаем плавное изменение
    if (pressDuration > 500) {
      static unsigned long lastHoldChange = 0;
      if (currentTime - lastHoldChange > 100) { // Каждые 100ms
        lastHoldChange = currentTime;
        desiredTemp -= TEMP_STEP;
        if (desiredTemp < TEMP_MIN) desiredTemp = TEMP_MIN;
        
        if (heaterEnabled) {
          targetTemp = desiredTemp;
          overshootDetected = false;
        }
        
        saveTemperatureToEEPROM();
      }
    }
  }
  
  // Сохраняем состояния для следующей итерации
  lastButtonUpState = buttonUpState;
  lastButtonDownState = buttonDownState;
}

// /////////////////////// ОБНОВЛЕНИЕ ТЕМПЕРАТУРЫ ///////////////////////

/**
 * Чтение и фильтрация температуры с термопары
 * Использует медианный фильтр 5 значений + экспоненциальное сглаживание
 */
void updateTemperature() {
  float newTemp = thermocouple.readCelsius();
  
  // Проверка на корректность значения
  if (isnan(newTemp) || newTemp < -50 || newTemp > 1000) {
    return; // Пропускаем некорректное значение
  }
  
  // Добавляем в кольцевой буфер истории
  tempHistory[tempHistoryIndex] = newTemp;
  tempHistoryIndex = (tempHistoryIndex + 1) % 5;
  
  // Копируем историю для сортировки
  float tempArray[5];
  for (int i = 0; i < 5; i++) {
    tempArray[i] = tempHistory[i];
  }
  
  // Сортировка пузырьком для медианы
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (tempArray[j] < tempArray[i]) {
        float temp = tempArray[i];
        tempArray[i] = tempArray[j];
        tempArray[j] = temp;
      }
    }
  }
  
  // Берем медиану (средний элемент отсортированного массива)
  float medianTemp = tempArray[2];
  
  // Экспоненциальное сглаживание (alpha = 0.7)
  float alpha = 0.7;
  currentTemp = currentTemp * alpha + medianTemp * (1.0 - alpha);
}

// /////////////////////// УПРАВЛЕНИЕ НАГРЕВАТЕЛЕМ ///////////////////////

/**
 * Алгоритм управления мощностью нагревателя
 * Адаптивно изменяет мощность в зависимости от:
 * - Разницы между текущей и целевой температурой
 * - Текущего режима работы (нагрев/поддержание/охлаждение)
 * - Состояния батареи
 */
void controlHeater() {
  unsigned long currentTime = millis();
  
  // Обновляем мощность не чаще чем каждые 300ms
  if (currentTime - lastHeatingUpdate < HEATING_UPDATE_INTERVAL) {
    return;
  }
  lastHeatingUpdate = currentTime;
  
  // ///////// ПРОВЕРКИ БЕЗОПАСНОСТИ /////////
  
  // Защита от перегрева
  if (currentTemp >= MAX_TEMPERATURE) {
    heaterEnabled = false;
    ledcWrite(0, 0);
    heatingPower = 0;
    lastHeatingPower = 0;
    currentMode = MODE_IDLE;
    lastActiveMode = MODE_IDLE;
    maintainSymbol = MODE_COOLING;
    firstTimeMaintain = false;
    return;
  }
  
  // Защита от разряда батареи
  if (batteryPercentage <= 0 || batteryVoltage < MIN_VOLTAGE_FOR_HEATING || criticalBattery) {
    heaterEnabled = false;
    ledcWrite(0, 0);
    heatingPower = 0;
    lastHeatingPower = 0;
    currentMode = MODE_IDLE;
    lastActiveMode = MODE_IDLE;
    maintainSymbol = MODE_COOLING;
    firstTimeMaintain = false;
    return;
  }
  
  // ///////// РАСЧЕТ ТЕКУЩЕЙ МОЩНОСТИ /////////
  
  float tempDiff = targetTemp - currentTemp;
  float absTempDiff = abs(tempDiff);
  
  // Адаптивная максимальная мощность в зависимости от расстояния до цели
  float adaptiveMaxPower = HEATER_POWER_MAX;
  if (absTempDiff > 100.0f) {
    adaptiveMaxPower = HEATER_POWER_MAX * 0.6f; // Дальше от цели - меньше макс мощность
  } else if (absTempDiff > 50.0f) {
    adaptiveMaxPower = HEATER_POWER_MAX * 0.7f;
  } else if (absTempDiff > 20.0f) {
    adaptiveMaxPower = HEATER_POWER_MAX * 0.8f;
  }
  
  // Адаптивная скорость изменения мощности
  float adaptiveMaxChange = 1.0f;
  if (absTempDiff > 50.0f) {
    adaptiveMaxChange = 0.7f; // Медленнее меняем при большом расстоянии
  }
  
  float newPower = 0.0;
  
  // Выбор мощности в зависимости от режима
  switch (currentMode) {
    case MODE_MAINTAIN:
      // Режим поддержания температуры
      if (currentTemp < targetTemp - 1.0) {
        newPower = HEATER_POWER_MAINTAIN;
      } else if (currentTemp > targetTemp + 0.5) {
        newPower = 0; // Слишком горячо - выключаем
      } else {
        newPower = HEATER_POWER_MAINTAIN / 3; // Небольшая мощность для точного поддержания
      }
      break;
      
    case MODE_HEATING:
      // Режим активного нагрева
      if (tempDiff > 100.0f) newPower = 25.0f;
      else if (tempDiff > 50.0f) newPower = 30.0f;
      else if (tempDiff > 20.0f) newPower = 35.0f;
      else if (tempDiff > 10.0f) newPower = 25.0f;
      else if (tempDiff > 5.0f) newPower = 15.0f;
      else if (tempDiff > 2.0f) newPower = 8.0f;
      else newPower = HEATER_POWER_MIN;
      
      // Плавное уменьшение мощности при приближении к цели
      if (tempDiff < 20.0f) {
        float factor = tempDiff / 20.0f;
        newPower = newPower * factor * 0.8f;
        if (newPower < HEATER_POWER_MIN) newPower = HEATER_POWER_MIN;
      }
      
      // Ограничение максимальной мощности
      if (newPower > adaptiveMaxPower) newPower = adaptiveMaxPower;
      break;
      
    case MODE_COOLING:
      // Режим охлаждения - выключаем нагрев
      newPower = 0;
      break;
      
    default:
      newPower = 0;
      break;
  }
  
  // ///////// ПЛАВНОЕ ИЗМЕНЕНИЕ МОЩНОСТИ /////////
  
  float powerChange = newPower - lastHeatingPower;
  
  // Ограничиваем скорость изменения мощности
  if (abs(powerChange) > adaptiveMaxChange) {
    if (powerChange > 0) {
      lastHeatingPower += adaptiveMaxChange; // Плавное увеличение
    } else {
      lastHeatingPower -= adaptiveMaxChange; // Плавное уменьшение
    }
  } else {
    lastHeatingPower = newPower;
  }
  
  // Ограничение значения мощности
  lastHeatingPower = constrain(lastHeatingPower, 0, adaptiveMaxPower);
  
  // ///////// УПРАВЛЕНИЕ ШИМ /////////
  
  // Конвертируем проценты в значение ШИМ (0-255 для 8 бит)
  int pwmValue = map(lastHeatingPower, 0, 100, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);
  ledcWrite(0, pwmValue);
  
  // Сохраняем текущую мощность для отладки
  heatingPower = lastHeatingPower;
}

// /////////////////////// ГРАФИЧЕСКИЕ ФУНКЦИИ ///////////////////////

/**
 * Рисует символ градуса Цельсия (маленький кружок)
 * @param x, y - координаты верхнего левого угла символа
 */
void drawDegree(int x, int y) {
  // Рисуем квадрат 5x5 пикселей с закругленными углами
  oled.drawPixel(x+1, y);
  oled.drawPixel(x+2, y);
  oled.drawPixel(x+3, y);
  oled.drawPixel(x+1, y+4);
  oled.drawPixel(x+2, y+4);
  oled.drawPixel(x+3, y+4);
  oled.drawPixel(x, y+1);
  oled.drawPixel(x, y+2);
  oled.drawPixel(x, y+3);
  oled.drawPixel(x+4, y+1);
  oled.drawPixel(x+4, y+2);
  oled.drawPixel(x+4, y+3);
  // Закрашиваем центр
  oled.drawPixel(x+1, y+1);
  oled.drawPixel(x+3, y+1);
  oled.drawPixel(x+1, y+3);
  oled.drawPixel(x+3, y+3);
}

/**
 * Рисует символ нагрева (огонь/пламя)
 * @param x, y - координаты центра символа
 */
void drawHeatingSymbol(int x, int y) {
  // Контур пламени
  oled.drawLine(x + 4, y, x + 2, y + 2);
  oled.drawLine(x + 2, y + 2, x + 1, y + 5);
  oled.drawLine(x + 1, y + 5, x + 2, y + 8);
  oled.drawLine(x + 4, y, x + 6, y + 2);
  oled.drawLine(x + 6, y + 2, x + 7, y + 5);
  oled.drawLine(x + 7, y + 5, x + 6, y + 8);
  // Внутренние языки пламени
  oled.drawLine(x + 2, y + 8, x + 3, y + 6);
  oled.drawLine(x + 3, y + 6, x + 4, y + 8);
  oled.drawLine(x + 4, y + 8, x + 5, y + 6);
  oled.drawLine(x + 5, y + 6, x + 6, y + 8);
  oled.drawLine(x + 3, y + 2, x + 4, y + 4);
  oled.drawLine(x + 4, y + 4, x + 5, y + 2);
  // Верхняя точка пламени
  oled.drawPixel(x + 4, y - 1);
}

/**
 * Рисует символ охлаждения (снежинка/звездочка)
 * @param x, y - координаты центра символа
 */
void drawCoolingSymbol(int x, int y) {
  // Вертикальная и горизонтальная линии
  oled.drawLine(x, y - 4, x, y + 4);
  oled.drawLine(x - 4, y, x + 4, y);
  // Диагональные линии
  oled.drawLine(x - 3, y - 3, x + 3, y + 3);
  oled.drawLine(x - 3, y + 3, x + 3, y - 3);
  // Центральная точка
  oled.drawPixel(x, y);
}

/**
 * Рисует индикатор батареи
 * @param x, y - координаты верхнего левого угла
 * @param percentage - процент заряда от 0 до 100
 */
void drawBatteryIndicator(int x, int y, int percentage) {
  // Контур батареи
  oled.drawFrame(x, y, 20, 10);
  // Полюс батареи
  oled.drawBox(x + 20, y + 3, 2, 4);
  
  // Расчет ширины заливки (18px максимум, минус отступы)
  int fillWidth = map(percentage, 0, 100, 0, 18);
  fillWidth = constrain(fillWidth, 0, 18);
  
  // Заливка в зависимости от процента
  if (fillWidth > 0) {
    oled.drawBox(x + 1, y + 1, fillWidth, 8);
  }
}

// /////////////////////// ФУНКЦИИ БЕЗОПАСНОСТИ ///////////////////////

/**
 * Проверка критических состояний системы
 * Вызывается каждый цикл loop()
 */
void safetyCheck() {
  // Защита от перегрева (если нагрев включен)
  if (currentTemp >= MAX_TEMPERATURE && heaterEnabled) {
    heaterEnabled = false;
    ledcWrite(0, 0);
    currentMode = MODE_IDLE;
    lastActiveMode = MODE_IDLE;
    maintainSymbol = MODE_COOLING;
    firstTimeMaintain = false;
  }
  
  // Защита от критического разряда батареи
  if (criticalBattery && heaterEnabled) {
    heaterEnabled = false;
    ledcWrite(0, 0);
    currentMode = MODE_IDLE;
    lastActiveMode = MODE_IDLE;
    maintainSymbol = MODE_COOLING;
    firstTimeMaintain = false;
  }
}

// /////////////////////// РАБОТА С ПАМЯТЬЮ EEPROM ///////////////////////

/**
 * Сохраняет текущую целевую температуру в EEPROM
 * Вызывается при каждом изменении температуры
 */
void saveTemperatureToEEPROM() {
  int tempToSave = (int)desiredTemp;
  if (tempToSave > 255) tempToSave = 255; // Ограничение для 1 байта
  EEPROM.write(0, tempToSave);
  EEPROM.commit(); // Сохраняем изменения
}

/**
 * Загружает сохраненную температуру из EEPROM
 * Вызывается при старте системы
 */
void loadSavedTemperature() {
  int savedValue = EEPROM.read(0);
  // Проверяем что значение в допустимом диапазоне
  if (savedValue >= TEMP_MIN && savedValue <= TEMP_MAX) {
    desiredTemp = savedValue;
  } else {
    desiredTemp = DEFAULT_TEMP; // Значение по умолчанию при ошибке
  }
}

// /////////////////////// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ///////////////////////

/**
 * Отображение сообщения об ошибке на дисплее
 * @param message - текст сообщения об ошибке
 */
void displayErrorMessage(const char* message) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(10, 30, message);
  oled.sendBuffer();
  delay(3000); // Показываем сообщение 3 секунды
}