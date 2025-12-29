// =================== БИБЛИОТЕКИ ===================
#include <U8g2lib.h>
#include <SPI.h>
#include <max6675.h>
#include <EEPROM.h>

// =================== ОПРЕДЕЛЕНИЕ ПИНОВ ===================
#define HEATER_PIN 12      // Пин управления нагревателем
#define THERMO_CLK 26      // Пин CLK для MAX6675
#define THERMO_CS  25      // Пин CS для MAX6675
#define THERMO_DO  33      // Пин DO для MAX6675
#define BUTTON_POWER 32    // Кнопка POWER (запуск/остановка)
#define BUTTON_UP    14    // Кнопка UP (увеличение температуры)
#define BUTTON_DOWN  27    // Кнопка DOWN (уменьшение температуры)
#define BATTERY_PIN 34     // АЦП для измерения напряжения батареи
#define OLED_CS     5      // Пин CS для OLED
#define OLED_DC     2      // Пин DC для OLED
#define OLED_RST    4      // Пин RST для OLED

// =================== НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ ===================

// ПАРАМЕТР ТЕМПЕРАТУРЫ: диапазон 0-30°C, шаг 5°C (для теста)
#define TEMP_MIN 0         // Минимальная температура (°C)
#define TEMP_MAX 30        // Максимальная температура (°C) - для теста
#define TEMP_STEP 5        // Шаг изменения температуры (°C)
#define DEFAULT_TEMP 30    // Температура по умолчанию (°C)

// ПАРАМЕТРЫ ТАЙМИНГА:
#define TEMP_UPDATE_INTERVAL 1000      // Интервал опроса датчика (мс) - 1 секунда
#define DISPLAY_UPDATE_INTERVAL 2000   // Интервал обновления дисплея (мс) - 2 секунды
#define PWM_CYCLE_TIME 2000           // Время цикла ШИМ (мс) - 2 секунды (50% скважность)
#define BUTTON_DEBOUNCE 50            // Время антидребезга кнопок (мс)
#define BUTTON_HOLD_TIME 2000         // Время удержания для включения/выключения (мс) - 2 секунды
#define DISPLAY_TIMEOUT 60000         // Таймаут дисплея при бездействии (мс) - 1 минута для теста
#define BUTTON_MULTICLICK_TIMEOUT 800 // Таймаут для многократных нажатий (мс) - 800 мс
#define BATTERY_UPDATE_INTERVAL 1000  // Обновление батареи каждую секунду

// ПАРАМЕТРЫ НАГРЕВАТЕЛЯ:
#define HEATER_POWER 100              // Мощность нагревателя (%)
#define MAX_TEMPERATURE 30.0          // Максимальная температура нагрева (°C)

// ПАРАМЕТРЫ БАТАРЕИ 3S Li-ion:
#define VOLTAGE_DIVIDER_RATIO 22.28   // Коэффициент делителя напряжения для R1=100к, R2=4.7к
#define ADC_MAX_VALUE 4095.0          // Максимальное значение АЦП
#define ADC_REF_VOLTAGE 3.3           // Опорное напряжение АЦП (V)
#define BATTERY_VOLTAGE_MAX 12.6      // Напряжение полного заряда (V)
#define BATTERY_VOLTAGE_MIN 9.0       // Напряжение полного разряда (V)
#define BATTERY_VOLTAGE_LOW 10.5      // Напряжение низкого заряда 25% (V)
#define BATTERY_VOLTAGE_CRITICAL 9.9  // Напряжение критического заряда (V)
#define LOW_BATTERY_WARNING_INTERVAL 600000 // Интервал предупреждения о низком заряде (мс) - 10 минут

// ФИЛЬТРАЦИЯ БАТАРЕИ:
#define FILTER_SAMPLES 20             // Количество выборок для фильтра
#define BATTERY_SMOOTH_FACTOR 0.1     // Коэффициент сглаживания (0-1)

// ПАРАМЕТРЫ ШИМ:
#define PWM_FREQUENCY 1000            // Частота ШИМ (Гц)
#define PWM_RESOLUTION 8              // Разрешение ШИМ (бит)

// =================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ===================
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// =================== ПЕРЕМЕННЫЕ ===================
float currentTemp = 0.0;              // Текущая температура (°C)
float desiredTemp = DEFAULT_TEMP;     // Желаемая температура (°C)
bool heaterEnabled = false;           // Флаг включения нагрева
bool systemOn = false;                // Флаг включения системы
bool displayOn = true;                // Флаг включения дисплея
bool heaterPWMState = false;          // Текущее состояние ШИМ (true = вкл, false = выкл)

// Переменные для батареи
float batteryVoltage = 0.0;           // Напряжение батареи (V)
float filteredVoltage = 0.0;          // Отфильтрованное напряжение (V)
int batteryPercentage = 100;          // Процент заряда батареи (%)
bool lowBatteryWarning = false;       // Флаг низкого заряда
bool criticalBattery = false;         // Флаг критического заряда
unsigned long lastBatteryWarning = 0; // Время последнего предупреждения о низком заряде
float voltageDividerRatio = VOLTAGE_DIVIDER_RATIO; // Реальный коэффициент делителя

// Буфер для фильтрации батареи
float voltageBuffer[FILTER_SAMPLES];
int bufferIndex = 0;

// Переменные для кнопок
int buttonPressCount = 0;             // Счетчик нажатий кнопки POWER
unsigned long buttonPressTime = 0;    // Время начала нажатия кнопки
unsigned long lastButtonRelease = 0;  // Время отпускания кнопки для многократных нажатий

// Переменные для таймингов
unsigned long lastTempUpdate = 0;     // Время последнего обновления температуры
unsigned long lastDisplayUpdate = 0;  // Время последнего обновления дисплея
unsigned long lastButtonCheck = 0;    // Время последней проверки кнопок
unsigned long lastActivity = 0;       // Время последней активности пользователя
unsigned long lastPWMCycle = 0;       // Время последнего изменения ШИМ
unsigned long lastBatteryCheck = 0;   // Время последней проверки батареи

// Флаги для режимов
bool calibrationMode = false;         // Флаг режима калибровки

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  Serial.println("========================================");
  Serial.println("СИСТЕМА НАГРЕВАТЕЛЯ - ИНИЦИАЛИЗАЦИЯ");
  Serial.println("========================================");
  
  // Настройка пинов
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  digitalWrite(HEATER_PIN, LOW);  // Гарантированно выключаем нагреватель
  
  // Настройка ШИМ для нагревателя
  ledcAttach(HEATER_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(HEATER_PIN, 0);
  
  // Инициализация OLED дисплея
  u8g2.begin();
  Serial.println("OLED дисплей инициализирован");
  
  // Инициализация EEPROM для сохранения настроек
  EEPROM.begin(128);
  
  // Загрузка сохраненной температуры из EEPROM
  loadSavedTemperature();
  
  // Загрузка калибровочного коэффициента из EEPROM
  loadCalibrationData();
  
  // Инициализация буфера фильтрации
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    voltageBuffer[i] = 0;
  }
  
  // Тест термопары
  delay(500);
  float testTemp = thermocouple.readCelsius();
  if(isnan(testTemp)) {
    Serial.println("ОШИБКА: Проверьте подключение термопары!");
    displayErrorMessage("THERMO ERROR");
  } else {
    currentTemp = testTemp;
    Serial.print("Начальная температура: ");
    Serial.print(currentTemp);
    Serial.println("°C");
  }
  
  // Начальное заполнение буфера батареи
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    updateBatteryStatus();
    delay(50);
  }
  
  lastActivity = millis();  // Записываем время начала работы
  
  Serial.println("\n=== ИНСТРУКЦИЯ ===");
  Serial.println("1. Удерживайте POWER 2 сек - ВКЛ/ВЫКЛ системы");
  Serial.println("2. Нажмите POWER 3 раза - ВКЛ/ВЫКЛ нагрева");
  Serial.println("3. Нажмите POWER 2 раза - ВКЛ/ВЫКЛ дисплея");
  Serial.println("4. Для калибровки: наберите 'CAL' в мониторе порта");
  Serial.println("5. Нагрев работает до 30°C (тестовый режим)");
  Serial.println("6. Батарея: пин 34 (ADC), параболический расчет");
  Serial.print("7. Коэффициент делителя: ");
  Serial.println(voltageDividerRatio, 2);
  Serial.println("========================================");
}

// =================== LOOP ===================
void loop() {
  unsigned long currentTime = millis();
  
  // Проверка команды калибровки из Serial
  checkSerialCommands();
  
  // Если в режиме калибровки - только калибровка
  if (calibrationMode) {
    calibrateVoltageDivider();
    return;
  }
  
  // 1. ОПРОС ТЕРМОПАРЫ - каждую секунду
  if (currentTime - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
    updateTemperature();
    lastTempUpdate = currentTime;
  }
  
  // 2. УПРАВЛЕНИЕ НАГРЕВАТЕЛЕМ С ШИМ - скважность 50%
  if (systemOn && heaterEnabled) {
    if (currentTime - lastPWMCycle >= PWM_CYCLE_TIME / 2) {  // Половина цикла
      controlHeater();
      lastPWMCycle = currentTime;
    }
  }
  
  // 3. ОБРАБОТКА КНОПОК - с антидребезгом
  if (currentTime - lastButtonCheck >= BUTTON_DEBOUNCE) {
    handleButtons();
    lastButtonCheck = currentTime;
  }
  
  // 4. ПРОВЕРКА БАТАРЕИ - каждую секунду
  if (currentTime - lastBatteryCheck >= BATTERY_UPDATE_INTERVAL) {
    updateBatteryStatus();
    
    // Проверка низкого заряда каждые 10 минут
    if (lowBatteryWarning && (currentTime - lastBatteryWarning >= LOW_BATTERY_WARNING_INTERVAL)) {
      displayLowBatteryWarning();
      lastBatteryWarning = currentTime;
    }
    
    lastBatteryCheck = currentTime;
  }
  
  // 5. ОБНОВЛЕНИЕ ДИСПЛЕЯ - каждые 2 секунды
  if (displayOn && systemOn) {
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      updateDisplay();
      lastDisplayUpdate = currentTime;
    }
  }
  
  // 6. ТАЙМАУТ ДИСПЛЕЯ - 1 минута бездействия
  if (systemOn && displayOn && (currentTime - lastActivity >= DISPLAY_TIMEOUT)) {
    displayOn = false;
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    Serial.println("Дисплей отключен по таймауту");
  }
  
  // 7. ОБРАБОТКА МНОГОКРАТНЫХ НАЖАТИЙ КНОПКИ
  if (buttonPressCount > 0 && (currentTime - lastButtonRelease >= BUTTON_MULTICLICK_TIMEOUT)) {
    processButtonMultiClicks();
  }
  
  // 8. ПРОВЕРКА БЕЗОПАСНОСТИ
  safetyCheck();
}

// =================== ФУНКЦИЯ: ПРОВЕРКА КОМАНД ИЗ SERIAL ===================
void checkSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "CAL" || command == "cal") {
      calibrationMode = true;
      Serial.println("\n=== ВХОД В РЕЖИМ КАЛИБРОВКИ ===");
      Serial.println("Отключите нагреватель для безопасности!");
      heaterEnabled = false;
      systemOn = false;
      ledcWrite(HEATER_PIN, 0);
    }
    else if (command == "INFO" || command == "info") {
      printBatteryInfo();
    }
    else if (command == "HELP" || command == "help") {
      printHelp();
    }
  }
}

// =================== ФУНКЦИЯ: КАЛИБРОВКА ДЕЛИТЕЛЯ НАПРЯЖЕНИЯ ===================
void calibrateVoltageDivider() {
  static int calibrationStep = 0;
  static float realVoltage = 0;
  static unsigned long calibrationStartTime = 0;
  
  switch (calibrationStep) {
    case 0: // Начало калибровки
      calibrationStartTime = millis();
      Serial.println("\n=== КАЛИБРОВКА ДЕЛИТЕЛЯ НАПРЯЖЕНИЯ ===");
      Serial.println("ШАГ 1: Подготовка");
      Serial.println("1. Подключите мультиметр к батарее");
      Serial.println("2. Измерьте реальное напряжение батареи");
      Serial.println("3. Убедитесь, что батарея стабильна");
      Serial.println("\nНапишите 'NEXT' для продолжения...");
      calibrationStep = 1;
      break;
      
    case 1: // Ожидание команды NEXT
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "NEXT" || cmd == "next") {
          Serial.println("\nШАГ 2: Ввод реального напряжения");
          Serial.println("Введите измеренное напряжение батареи в вольтах");
          Serial.println("Например: 12.34");
          Serial.print("> ");
          calibrationStep = 2;
        }
      }
      break;
      
    case 2: // Ввод реального напряжения
      if (Serial.available()) {
        realVoltage = Serial.parseFloat();
        Serial.println(realVoltage, 2);
        
        if (realVoltage < 9.0 || realVoltage > 13.0) {
          Serial.println("ОШИБКА: Напряжение должно быть 9.0V - 13.0V");
          Serial.println("Повторите ввод:");
          Serial.print("> ");
        } else {
          Serial.print("Принято: ");
          Serial.print(realVoltage, 2);
          Serial.println("V");
          
          Serial.println("\nШАГ 3: Измерение ADC...");
          
          // Измеряем ADC 50 раз для усреднения
          const int numSamples = 50;
          long sum = 0;
          for (int i = 0; i < numSamples; i++) {
            sum += analogRead(BATTERY_PIN);
            delay(10);
            if (i % 10 == 0) Serial.print(".");
          }
          Serial.println();
          
          int rawValue = sum / numSamples;
          float adcVoltage = (rawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
          
          // Рассчитываем коэффициент делителя
          voltageDividerRatio = realVoltage / adcVoltage;
          
          Serial.println("\n=== РЕЗУЛЬТАТЫ КАЛИБРОВКИ ===");
          Serial.print("ADC raw значение: ");
          Serial.println(rawValue);
          Serial.print("Напряжение на ADC: ");
          Serial.print(adcVoltage, 4);
          Serial.println("V");
          Serial.print("Реальное напряжение батареи: ");
          Serial.print(realVoltage, 2);
          Serial.println("V");
          Serial.print("РАССЧИТАННЫЙ КОЭФФИЦИЕНТ: ");
          Serial.println(voltageDividerRatio, 4);
          
          // Проверка безопасности
          if (adcVoltage > 3.3) {
            Serial.println("ВНИМАНИЕ: Напряжение на ADC > 3.3V! Риск повреждения ESP32!");
          } else if (adcVoltage > 2.5) {
            Serial.println("ВНИМАНИЕ: Напряжение на ADC > 2.5V! Делитель слишком мал!");
          } else if (adcVoltage < 0.1) {
            Serial.println("ВНИМАНИЕ: Напряжение на ADC < 0.1V! Делитель слишком велик!");
          } else {
            Serial.println("БЕЗОПАСНО: Напряжение в допустимых пределах");
          }
          
          Serial.println("\nШАГ 4: Сохранение");
          Serial.println("Введите 'SAVE' для сохранения или 'CANCEL' для отмены");
          Serial.print("> ");
          calibrationStep = 3;
        }
      }
      break;
      
    case 3: // Сохранение или отмена
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "SAVE" || cmd == "save") {
          // Сохраняем в EEPROM
          EEPROM.put(10, voltageDividerRatio); // Адрес 10 для коэффициента
          EEPROM.commit();
          
          Serial.println("Коэффициент сохранен в EEPROM!");
          Serial.print("Новый коэффициент: ");
          Serial.println(voltageDividerRatio, 4);
          
          // Тестовое измерение с новым коэффициентом
          Serial.println("\nТестовое измерение с новым коэффициентом:");
          updateBatteryStatus();
          
        } else if (cmd == "CANCEL" || cmd == "cancel") {
          Serial.println("Калибровка отменена. Коэффициент не сохранен.");
        }
        
        // Выход из режима калибровки
        calibrationMode = false;
        calibrationStep = 0;
        Serial.println("\n=== ВЫХОД ИЗ РЕЖИМА КАЛИБРОВКИ ===");
      }
      break;
  }
  
  // Таймаут калибровки (5 минут)
  if (calibrationMode && (millis() - calibrationStartTime > 300000)) {
    calibrationMode = false;
    calibrationStep = 0;
    Serial.println("\nТАЙМАУТ: Калибровка прервана по времени");
  }
}

// =================== ФУНКЦИЯ: ЗАГРУЗКА КАЛИБРОВОЧНЫХ ДАННЫХ ===================
void loadCalibrationData() {
  float savedRatio;
  EEPROM.get(10, savedRatio);
  
  // Проверяем, что значение в разумных пределах
  if (savedRatio > 10.0 && savedRatio < 100.0) {
    voltageDividerRatio = savedRatio;
    Serial.print("Загружен калибровочный коэффициент: ");
    Serial.println(voltageDividerRatio, 4);
  } else {
    Serial.print("Используется коэффициент по умолчанию: ");
    Serial.println(voltageDividerRatio, 2);
  }
}

// =================== ФУНКЦИЯ: ВЫВОД ИНФОРМАЦИИ О БАТАРЕЕ ===================
void printBatteryInfo() {
  Serial.println("\n=== ИНФОРМАЦИЯ О БАТАРЕЕ ===");
  Serial.print("Коэффициент делителя: ");
  Serial.println(voltageDividerRatio, 4);
  
  // Сырые данные ADC
  int raw = analogRead(BATTERY_PIN);
  Serial.print("ADC raw (одно измерение): ");
  Serial.println(raw);
  
  // Расчетное напряжение на ADC
  float adcV = (raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  Serial.print("Напряжение на ADC: ");
  Serial.print(adcV, 4);
  Serial.println("V");
  
  // Расчетное напряжение батареи
  float calcV = adcV * voltageDividerRatio;
  Serial.print("Расчетное напряжение батареи: ");
  Serial.print(calcV, 2);
  Serial.println("V");
  
  // Процент заряда (параболический расчет)
  int perc = calculateBatteryPercentageNonlinear(calcV);
  Serial.print("Процент заряда (параболический): ");
  Serial.print(perc);
  Serial.println("%");
  
  // Безопасность
  if (adcV > 3.3) {
    Serial.println("ОПАСНО: Напряжение на ADC > 3.3V!");
  } else if (adcV > 3.0) {
    Serial.println("ПРЕДУПРЕЖДЕНИЕ: Напряжение на ADC близко к 3.3V");
  }
  
  Serial.println("==============================");
}

// =================== ФУНКЦИЯ: ВЫВОД СПРАВКИ ===================
void printHelp() {
  Serial.println("\n=== КОМАНДЫ SERIAL ===");
  Serial.println("CAL    - Калибровка делителя напряжения");
  Serial.println("INFO   - Информация о батарее");
  Serial.println("HELP   - Эта справка");
  Serial.println("\n=== УПРАВЛЕНИЕ КНОПКАМИ ===");
  Serial.println("Удержание POWER 2 сек - ВКЛ/ВЫКЛ системы");
  Serial.println("Тройное нажатие POWER - ВКЛ/ВЫКЛ нагрева");
  Serial.println("Двойное нажатие POWER - ВКЛ/ВЫКЛ дисплея");
  Serial.println("UP/DOWN - Изменение температуры");
  Serial.println("==============================");
}

// =================== ФУНКЦИЯ: ОБНОВЛЕНИЕ ТЕМПЕРАТУРЫ ===================
void updateTemperature() {
  float newTemp = thermocouple.readCelsius();
  
  if (isnan(newTemp) || newTemp < -50 || newTemp > 400) {
    Serial.println("ОШИБКА: Неверные показания термопары!");
    if (heaterEnabled) {
      heaterEnabled = false;
      ledcWrite(HEATER_PIN, 0);
      Serial.println("Нагрев отключен из-за ошибки термопары");
    }
    return;
  }
  
  currentTemp = newTemp;
}

// =================== ФУНКЦИЯ: УПРАВЛЕНИЕ НАГРЕВАТЕЛЕМ ===================
void controlHeater() {
  // Если нагрев не включен или система выключена - выключаем
  if (!systemOn || !heaterEnabled) {
    ledcWrite(HEATER_PIN, 0);
    heaterPWMState = false;
    return;
  }
  
  // Проверка достижения максимальной температуры
  if (currentTemp >= MAX_TEMPERATURE) {
    ledcWrite(HEATER_PIN, 0);
    heaterPWMState = false;
    heaterEnabled = false;  // Отключаем нагрев при достижении
    Serial.print("Достигнуто ");
    Serial.print(MAX_TEMPERATURE);
    Serial.println("°C! Нагрев отключен.");
    return;
  }
  
  // Проверка критического уровня батареи
  if (criticalBattery) {
    ledcWrite(HEATER_PIN, 0);
    heaterPWMState = false;
    heaterEnabled = false;
    Serial.println("Нагрев отключен: критический уровень батареи!");
    return;
  }
  
  // ШИМ СКВАЖНОСТЬЮ 50%: переключаем состояние каждую секунду
  heaterPWMState = !heaterPWMState;
  
  if (heaterPWMState) {
    // Включаем нагреватель на полную мощность
    int pwmValue = map(HEATER_POWER, 0, 100, 0, 255);
    ledcWrite(HEATER_PIN, pwmValue);
  } else {
    // Выключаем нагреватель
    ledcWrite(HEATER_PIN, 0);
  }
}

// =================== ФУНКЦИЯ: ОБРАБОТКА КНОПОК ===================
void handleButtons() {
  // ОБРАБОТКА КНОПКИ POWER
  if (digitalRead(BUTTON_POWER) == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
      lastActivity = millis();  // Сбрасываем таймер бездействия
      
      // Если дисплей выключен - включаем его
      if (systemOn && !displayOn) {
        displayOn = true;
        updateDisplay();
      }
    }
    
    // УДЕРЖАНИЕ 2 СЕКУНДЫ - ВКЛ/ВЫКЛ СИСТЕМЫ
    if (millis() - buttonPressTime >= BUTTON_HOLD_TIME) {
      systemOn = !systemOn;
      displayOn = true;
      lastActivity = millis();
      
      if (systemOn) {
        Serial.println("=== СИСТЕМА ВКЛЮЧЕНА ===");
      } else {
        heaterEnabled = false;
        ledcWrite(HEATER_PIN, 0);
        Serial.println("=== СИСТЕМА ВЫКЛЮЧЕНА ===");
      }
      updateDisplay();
      
      buttonPressTime = 0;
      delay(300);  // Задержка для предотвращения повторного срабатывания
      return;
    }
  } else {
    // КНОПКА ОТПУЩЕНА
    if (buttonPressTime > 0) {
      unsigned long pressDuration = millis() - buttonPressTime;
      
      // КОРОТКОЕ НАЖАТИЕ (менее 500 мс)
      if (pressDuration < 500 && pressDuration > 50) {
        buttonPressCount++;
        lastButtonRelease = millis();
        Serial.print("Нажатие POWER: ");
        Serial.println(buttonPressCount);
        
        buttonPressTime = 0;
        return;
      }
      
      buttonPressTime = 0;
    }
  }
  
  // ОБРАБОТКА КНОПОК UP/DOWN (только при включенной системе)
  if (systemOn) {
    // КНОПКА UP - УВЕЛИЧЕНИЕ ТЕМПЕРАТУРЫ
    if (digitalRead(BUTTON_UP) == LOW && (millis() - lastActivity > BUTTON_DEBOUNCE)) {
      desiredTemp += TEMP_STEP;
      if (desiredTemp > TEMP_MAX) desiredTemp = TEMP_MAX;
      lastActivity = millis();
      saveTemperatureToEEPROM();  // Сохраняем в EEPROM
      Serial.print("Желаемая температура установлена: ");
      Serial.print(desiredTemp);
      Serial.println("°C");
      updateDisplay();
      delay(BUTTON_DEBOUNCE);
    }
    
    // КНОПКА DOWN - УМЕНЬШЕНИЕ ТЕМПЕРАТУРЫ
    if (digitalRead(BUTTON_DOWN) == LOW && (millis() - lastActivity > BUTTON_DEBOUNCE)) {
      desiredTemp -= TEMP_STEP;
      if (desiredTemp < TEMP_MIN) desiredTemp = TEMP_MIN;
      lastActivity = millis();
      saveTemperatureToEEPROM();  // Сохраняем в EEPROM
      Serial.print("Желаемая температура установлена: ");
      Serial.print(desiredTemp);
      Serial.println("°C");
      updateDisplay();
      delay(BUTTON_DEBOUNCE);
    }
  }
}

// =================== ФУНКЦИЯ: ОБРАБОТКА МНОГОКРАТНЫХ НАЖАТИЙ ===================
void processButtonMultiClicks() {
  Serial.print("Обработка многократных нажатий: ");
  Serial.println(buttonPressCount);
  
  // ТРОЙНОЕ НАЖАТИЕ - ВКЛ/ВЫКЛ НАГРЕВА
  if (buttonPressCount == 3 && systemOn) {
    heaterEnabled = !heaterEnabled;
    lastActivity = millis();
    
    if (heaterEnabled) {
      Serial.println("=== НАГРЕВ ВКЛЮЧЕН (тройное нажатие)! ===");
      Serial.print("Будет работать до ");
      Serial.print(MAX_TEMPERATURE);
      Serial.println("°C");
      lastPWMCycle = millis();
      heaterPWMState = true;
    } else {
      Serial.println("=== НАГРЕВ ВЫКЛЮЧЕН (тройное нажатие) ===");
      ledcWrite(HEATER_PIN, 0);
    }
    updateDisplay();
  }
  
  // ДВОЙНОЕ НАЖАТИЕ - ВКЛ/ВЫКЛ ДИСПЛЕЯ
  else if (buttonPressCount == 2 && systemOn) {
    displayOn = !displayOn;
    lastActivity = millis();
    
    if (displayOn) {
      Serial.println("Дисплей включен (двойное нажатие)");
      updateDisplay();
    } else {
      Serial.println("Дисплей выключен (двойное нажатие)");
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
  }
  
  // Сброс счетчика
  buttonPressCount = 0;
}

// =================== ФУНКЦИЯ: ОБНОВЛЕНИЕ СТАТУСА БАТАРЕИ ===================
void updateBatteryStatus() {
  // Усреднение 10 измерений для уменьшения шума
  const int numSamples = 10;
  long sum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(100);
  }
  
  int rawValue = sum / numSamples;
  
  // Проверка на корректность ADC
  if (rawValue < 10 || rawValue > 4095) {
    Serial.println("ОШИБКА: Некорректное значение ADC!");
    return;
  }
  
  float adcVoltage = (rawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  
  // Используем калибровочный коэффициент
  batteryVoltage = adcVoltage * voltageDividerRatio;
  
  // Фильтрация скользящим средним
  voltageBuffer[bufferIndex] = batteryVoltage;
  bufferIndex = (bufferIndex + 1) % FILTER_SAMPLES;
  
  float sumFiltered = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sumFiltered += voltageBuffer[i];
  }
  filteredVoltage = sumFiltered / FILTER_SAMPLES;
  
  // Расчет процента заряда - ПАРАБОЛИЧЕСКИЙ МЕТОД
  batteryPercentage = calculateBatteryPercentageNonlinear(filteredVoltage);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // Проверка уровней заряда для 3S Li-ion
  lowBatteryWarning = (filteredVoltage <= BATTERY_VOLTAGE_LOW);
  criticalBattery = (filteredVoltage <= BATTERY_VOLTAGE_CRITICAL);
  
  // Отладочный вывод каждые 10 секунд
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 10000) {
    Serial.println("\n=== ИНФОРМАЦИЯ О БАТАРЕЕ ===");
    Serial.print("ADC raw: ");
    Serial.print(rawValue);
    Serial.print(" (");
    Serial.print((rawValue / ADC_MAX_VALUE) * 100, 1);
    Serial.println("% от максимума)");
    
    Serial.print("ADC voltage: ");
    Serial.print(adcVoltage, 3);
    Serial.println("V");
    
    Serial.print("Батарея (фильтрованное): ");
    Serial.print(filteredVoltage, 2);
    Serial.print("V (");
    Serial.print(batteryPercentage);
    Serial.println("%)");
    
    Serial.print("Коэффициент делителя: ");
    Serial.println(voltageDividerRatio, 2);
    
    // Проверка безопасности ADC
    if (adcVoltage > 3.0) {
      Serial.println("ВНИМАНИЕ: Напряжение на ADC > 3.0V! Проверьте делитель!");
    }
    
    Serial.println("=============================");
    lastDebug = millis();
  }
}

// =================== ФУНКЦИЯ: ПАРАБОЛИЧЕСКИЙ РАСЧЕТ ПРОЦЕНТА ЗАРЯДА ===================
int calculateBatteryPercentageNonlinear(float voltage) {
  // Для 3S Li-ion (3 элемента по 3.0-4.2V = 9.0V-12.6V)
  // Параболический расчет с учетом кривой разряда Li-ion
  
  float v = voltage;
  
  if (v >= 12.6) return 100;  // Полностью заряжена
  if (v <= 9.0) return 0;     // Полностью разряжена
  
  // Параболические участки кривой разряда Li-ion
  
  // 1. Участок 12.6V - 12.0V (100%-80%) - Быстрый спад в начале
  if (v >= 12.0) {
    float x = (v - 12.0) / (12.6 - 12.0); // 0-1
    // Парабола: y = 20*x^2 + 80
    return (int)(20 * x * x + 80);
  }
  
  // 2. Участок 12.0V - 11.4V (80%-40%) - Более линейный
  if (v >= 11.4) {
    float x = (v - 11.4) / (12.0 - 11.4); // 0-1
    // Парабола: y = -20*x^2 + 60*x + 40
    return (int)(-20 * x * x + 60 * x + 40);
  }
  
  // 3. Участок 11.4V - 10.5V (40%-10%) - Быстрый спад в конце
  if (v >= 10.5) {
    float x = (v - 10.5) / (11.4 - 10.5); // 0-1
    // Парабола: y = 30*x^0.7 + 10
    return (int)(30 * pow(x, 0.7) + 10);
  }
  
  // 4. Участок 10.5V - 9.0V (10%-0%) - Очень быстрый спад
  float x = (v - 9.0) / (10.5 - 9.0); // 0-1
  // Квадратный корень для резкого спада
  return (int)(10 * sqrt(x));
}

// =================== ФУНКЦИЯ: ТАБЛИЧНЫЙ РАСЧЕТ ПРОЦЕНТА ЗАРЯДА ===================
int calculateBatteryPercentage(float voltage) {
  // Используем параболический расчет
  return calculateBatteryPercentageNonlinear(voltage);
}

// =================== ФУНКЦИЯ: ОБНОВЛЕНИЕ ДИСПЛЕЯ ===================
void updateDisplay() {
  if (!displayOn) return;
  
  u8g2.clearBuffer();
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 128, 64);
  u8g2.setDrawColor(1);
  
  // СТАТУС СИСТЕМЫ (верхний левый угол)
  u8g2.setFont(u8g2_font_micro_tr);
  if (!systemOn) {
    u8g2.drawStr(5, 7, "OFF");
  } else if (heaterEnabled) {
    u8g2.drawStr(5, 7, "HEATING");
  } else {
    u8g2.drawStr(5, 7, "STANDBY");
  }
  
  // ТЕКУЩАЯ ТЕМПЕРАТУРА (левая часть)
  u8g2.setFont(u8g2_font_logisoso28_tn);
  char tempDisplay[6];
  sprintf(tempDisplay, "%d", (int)currentTemp);
  int tempWidth = u8g2.getStrWidth(tempDisplay);
  int tempX = (128 - tempWidth) / 2 - 20;
  int tempY = 45;
  u8g2.drawStr(tempX, tempY, tempDisplay);
  
  // СИМВОЛ ГРАДУСА ДЛЯ ТЕКУЩЕЙ ТЕМПЕРАТУРЫ
  drawTinyDegree(tempX + tempWidth + 3, tempY - 28);
  
  // ИКОНКА НАГРЕВА (если включен) - С МИГАНИЕМ
  int symbolX = tempX + tempWidth + 15;
  if (systemOn && heaterEnabled) {
    // Делаем огонек мигающим
    static bool blinkState = false;
    static unsigned long lastBlink = 0;
    
    if (millis() - lastBlink > 500) {  // Мигаем каждые 500 мс
      blinkState = !blinkState;
      lastBlink = millis();
    }
    
    if (blinkState) {
      drawHeatingSymbol(symbolX, tempY + 10);
    }
  }
  
  // СТРЕЛКА ОТ ТЕКУЩЕЙ К ЖЕЛАЕМОЙ ТЕМПЕРАТУРЕ
  int arrowStartX = tempX + tempWidth + 10;
  int arrowEndX = 85;
  int arrowY = tempY - 10;
  u8g2.drawLine(arrowStartX, arrowY, arrowEndX, arrowY);
  u8g2.drawLine(arrowEndX, arrowY, arrowEndX - 3, arrowY - 2);
  u8g2.drawLine(arrowEndX, arrowY, arrowEndX - 3, arrowY + 2);
  
  // ЖЕЛАЕМАЯ ТЕМПЕРАТУРА (правая часть)
  u8g2.setFont(u8g2_font_logisoso18_tn);
  char targetDisplay[5];
  sprintf(targetDisplay, "%d", (int)desiredTemp);
  int targetWidth = u8g2.getStrWidth(targetDisplay);
  int targetX = 95;
  int targetY = tempY;
  u8g2.drawStr(targetX, targetY, targetDisplay);
  
  // СИМВОЛ ГРАДУСА ДЛЯ ЖЕЛАЕМОЙ ТЕМПЕРАТУРЫ
  drawTinyDegree(targetX + targetWidth + 3, targetY - 20);
  
  // ИНДИКАТОР БАТАРЕИ (правый верхний угол)
  drawBatteryIndicator(105, 3, batteryPercentage);
  
  // ПРОЦЕНТ ЗАРЯДА БАТАРЕИ
  u8g2.setFont(u8g2_font_helvB08_tr);
  char batStr[6];
  sprintf(batStr, "%d%%", batteryPercentage);
  int batWidth = u8g2.getStrWidth(batStr);
  int batX = 105 - batWidth - 5;
  u8g2.drawStr(batX, 12, batStr);
  
  // ВОСКЛИЦАТЕЛЬНЫЙ ЗНАК ПРИ НИЗКОМ ЗАРЯДЕ (25% и ниже)
  if (lowBatteryWarning) {
    u8g2.drawStr(batX - 15, 12, "!");
  }
  
  // НИЖНИЙ РЕГИСТР (информация)
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(5, 62, "CELSING");   // Логотип слева снизу
  u8g2.drawStr(115, 22, "SET");     // Надпись "SET" справа сверху
  
  // Информация о батарее внизу (дополнительно)
  char voltStr[10];
  sprintf(voltStr, "%.1fV", filteredVoltage);
  u8g2.drawStr(80, 62, voltStr);
  
  u8g2.sendBuffer();
}

// =================== ФУНКЦИЯ: ИНДИКАТОР БАТАРЕИ ===================
void drawBatteryIndicator(int x, int y, int percentage) {
  // Контур батареи
  u8g2.drawFrame(x, y, 20, 10);
  u8g2.drawBox(x + 20, y + 3, 2, 4);
  
  // Уровень заряда
  int fillWidth = map(percentage, 0, 100, 0, 18);
  fillWidth = constrain(fillWidth, 0, 18);
  
  // Цвет в зависимости от уровня заряда
  if (percentage > 50) {
    u8g2.setDrawColor(1);  // Нормальный заряд
  } else if (percentage > 25) {
    u8g2.setDrawColor(1);  // Средний заряд
  } else {
    u8g2.setDrawColor(1);  // Низкий заряд
  }
  
  u8g2.drawBox(x + 1, y + 1, fillWidth, 8);
  u8g2.setDrawColor(1);  // Возвращаем белый цвет
}

// =================== ФУНКЦИЯ: БЕЗОПАСНОСТЬ ===================
void safetyCheck() {
  // ПРОВЕРКА ПРЕВЫШЕНИЯ ТЕМПЕРАТУРЫ
  if (currentTemp > MAX_TEMPERATURE + 2.0) {
    heaterEnabled = false;
    ledcWrite(HEATER_PIN, 0);
    Serial.println("БЕЗОПАСНОСТЬ: Критическое превышение температуры!");
    updateDisplay();
  }
  
  // АВАРИЙНОЕ ОТКЛЮЧЕНИЕ ПРИ КРИТИЧЕСКОМ УРОВНЕ БАТАРЕИ
  if (criticalBattery && systemOn) {
    heaterEnabled = false;
    systemOn = false;
    ledcWrite(HEATER_PIN, 0);
    Serial.println("ЭКСТРЕННОЕ ОТКЛЮЧЕНИЕ: Критический уровень батареи!");
    updateDisplay();
  }
  
  // ПРОВЕРКА БЕЗОПАСНОСТИ ДЕЛИТЕЛЯ НАПРЯЖЕНИЯ
  static unsigned long lastSafetyCheck = 0;
  if (millis() - lastSafetyCheck >= 10000) {
    int raw = analogRead(BATTERY_PIN);
    float adcV = (raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    
    if (adcV > 3.3) {
      Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Напряжение на ADC > 3.3V! Отключите питание!");
      // Аварийное отключение всего
      heaterEnabled = false;
      systemOn = false;
      ledcWrite(HEATER_PIN, 0);
    }
    
    lastSafetyCheck = millis();
  }
}

// =================== ФУНКЦИЯ: ПРЕДУПРЕЖДЕНИЕ О НИЗКОМ ЗАРЯДЕ ===================
void displayLowBatteryWarning() {
  if (!displayOn) {
    displayOn = true;
    lastActivity = millis();
  }
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(10, 25, "LOW BATTERY!");
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(15, 45, "Charge: ");
  u8g2.setCursor(70, 45);
  u8g2.print(batteryPercentage);
  u8g2.print("%");
  
  // Показываем напряжение
  u8g2.setCursor(15, 55);
  u8g2.print(filteredVoltage, 1);
  u8g2.print("V");
  
  u8g2.drawStr(25, 60, "10 min reminder");
  u8g2.sendBuffer();
  
  Serial.print("ПРЕДУПРЕЖДЕНИЕ: Низкий заряд батареи! ");
  Serial.print(batteryPercentage);
  Serial.print("%, ");
  Serial.print(filteredVoltage, 1);
  Serial.println("V");
  
  delay(3000);  // Показываем предупреждение 3 секунды
  
  if (systemOn) {
    updateDisplay();
  }
}

// =================== ФУНКЦИЯ: СОХРАНЕНИЕ ТЕМПЕРАТУРЫ В EEPROM ===================
void saveTemperatureToEEPROM() {
  EEPROM.write(0, (byte)desiredTemp);
  EEPROM.commit();
  Serial.println("Температура сохранена в EEPROM");
}

// =================== ФУНКЦИЯ: ЗАГРУЗКА ТЕМПЕРАТУРЫ ИЗ EEPROM ===================
void loadSavedTemperature() {
  byte savedTemp = EEPROM.read(0);
  if (savedTemp >= TEMP_MIN && savedTemp <= TEMP_MAX) {
    desiredTemp = savedTemp;
    Serial.print("Загружена сохраненная температура: ");
    Serial.print(desiredTemp);
    Serial.println("°C");
  }
}

// =================== ФУНКЦИЯ: ОШИБКА ТЕРМОПАРЫ ===================
void displayErrorMessage(const char* message) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(20, 30, message);
  u8g2.sendBuffer();
  delay(2000);
}

// =================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ ОТРИСОВКИ ===================

// РИСОВАНИЕ СИМВОЛА НАГРЕВА (ОГОНЬКА)
void drawHeatingSymbol(int x, int y) {
  u8g2.drawLine(x + 4, y, x + 2, y + 2);
  u8g2.drawLine(x + 2, y + 2, x + 1, y + 5);
  u8g2.drawLine(x + 1, y + 5, x + 2, y + 8);
  u8g2.drawLine(x + 4, y, x + 6, y + 2);
  u8g2.drawLine(x + 6, y + 2, x + 7, y + 5);
  u8g2.drawLine(x + 7, y + 5, x + 6, y + 8);
  u8g2.drawLine(x + 2, y + 8, x + 3, y + 6);
  u8g2.drawLine(x + 3, y + 6, x + 4, y + 8);
  u8g2.drawLine(x + 4, y + 8, x + 5, y + 6);
  u8g2.drawLine(x + 5, y + 6, x + 6, y + 8);
  u8g2.drawLine(x + 3, y + 2, x + 4, y + 4);
  u8g2.drawLine(x + 4, y + 4, x + 5, y + 2);
  u8g2.drawPixel(x + 4, y - 1);
}

// РИСОВАНИЕ МАЛЕНЬКОГО СИМВОЛА ГРАДУСА
void drawTinyDegree(int x, int y) {
  u8g2.drawPixel(x+1, y);
  u8g2.drawPixel(x+2, y);
  u8g2.drawPixel(x+3, y);
  u8g2.drawPixel(x+1, y+4);
  u8g2.drawPixel(x+2, y+4);
  u8g2.drawPixel(x+3, y+4);
  u8g2.drawPixel(x, y+1);
  u8g2.drawPixel(x, y+2);
  u8g2.drawPixel(x, y+3);
  u8g2.drawPixel(x+4, y+1);
  u8g2.drawPixel(x+4, y+2);
  u8g2.drawPixel(x+4, y+3);
  u8g2.drawPixel(x+1, y+1);
  u8g2.drawPixel(x+3, y+1);
  u8g2.drawPixel(x+1, y+3);
  u8g2.drawPixel(x+3, y+3);
}