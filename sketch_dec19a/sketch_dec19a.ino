#include <U8g2lib.h>
#include <SPI.h>
#include <max6675.h>
#include <EEPROM.h>

// =================== ПИНЫ ПОДКЛЮЧЕНИЯ ===================
#define HEATER_PIN 13      // Управление MOSFET нагревателя (D13)
#define THERMO_CLK 26      // Тактовый пин термопары MAX6675
#define THERMO_CS  25      // Пин выбора термопары
#define THERMO_DO  33      // Пин данных термопары
#define BUTTON_POWER 32    // Главная кнопка (вкл/выкл, режимы)
#define BUTTON_UP    14    // Кнопка увеличения температуры
#define BUTTON_DOWN  27    // Кнопка уменьшения температуры
#define BATTERY_PIN 34     // Измерение напряжения батареи (ADC1_CH6)
#define OLED_CS     5      // Пин CS OLED дисплея
#define OLED_DC     2      // Пин DC OLED
#define OLED_RST    4      // Пин сброса OLED

// =================== НАСТРОЙКИ ТЕМПЕРАТУРЫ ===================
#define TEMP_MIN 0         // Минимальная температура (°C)
#define TEMP_MAX 250       // Максимальная температура (°C)
#define TEMP_STEP 1        // Шаг изменения температуры кнопками (°C)
#define DEFAULT_TEMP 200   // Температура по умолчанию (°C)

// =================== ИНТЕРВАЛЫ ВРЕМЕНИ ===================
#define TEMP_UPDATE_INTERVAL 1000    // Опрос датчика температуры (мс)
#define DISPLAY_UPDATE_INTERVAL 1000 // Обновление дисплея (мс)
#define BUTTON_DEBOUNCE 100          // Задержка для подавления дребезга кнопок (мс)
#define BUTTON_HOLD_TIME 2000        // Удержание для включения/выключения (мс)
#define DISPLAY_TIMEOUT 5000         // Таймаут отключения дисплея при бездействии (мс)
#define BATTERY_UPDATE_INTERVAL 5000 // Интервал обновления батареи (мс)
#define DISPLAY_TARGET_REACHED_TIME 5000 // Время показа температуры после достижения цели

// =================== НАСТРОЙКИ НАГРЕВА ===================
#define MAX_TEMPERATURE 250.0        // Максимальная безопасная температура (°C)
#define HEATER_POWER_MAX 40          // Максимальная мощность нагрева (%) - УМЕНЬШЕНО!
#define HEATER_POWER_MAINTAIN 8      // Мощность для поддержания температуры (%) - УМЕНЬШЕНО!
#define HEATER_POWER_MIN 3           // Минимальная мощность нагрева (%) - УМЕНЬШЕНО!
#define PWM_FREQUENCY 10             // Частота ШИМ для плавного нагрева (Гц)
#define PWM_RESOLUTION 8             // Разрешение ШИМ (бит)
#define PWM_CHANNEL 0                // Канал ШИМ
#define HEATING_UPDATE_INTERVAL 300  // Интервал обновления мощности нагрева (мс) - УМЕНЬШЕНО!

// =================== НАСТРОЙКИ БАТАРЕИ ===================
// ВНИМАНИЕ: ПЕРЕПРОВЕРЬТЕ ЭТИ КОЭФФИЦИЕНТЫ!
// Если делитель из 4.2В, то скорее всего коэффициент около 2.0-2.5
// Пример расчета: если делитель 1:2, то коэффициент = 2.0
// Если используете преобразователь, возможно напряжение уже понижено
#define VOLTAGE_DIVIDER_COEFF 2.5    // ПЕРЕПРОВЕРЬТЕ ЭТОТ КОЭФФИЦИЕНТ!
#define ADC_MAX_VALUE 4095.0         // Максимальное значение ADC ESP32
#define ADC_REF_VOLTAGE 3.3          // Опорное напряжение ADC (В)
#define BATTERY_FULL_VOLTAGE 4.2     // Напряжение полного заряда (В)
#define BATTERY_EMPTY_VOLTAGE 3.3    // Напряжение разряженной батареи (В)
#define BATTERY_LOW_THRESHOLD 25     // Порог низкого заряда (%)
#define MIN_VOLTAGE_FOR_HEATING 3.50 // Минимальное напряжение для работы нагрева (В)

// =================== ИНИЦИАЛИЗАЦИЯ БИБЛИОТЕК ===================
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// =================== ПЕРЕМЕННЫЕ СИСТЕМЫ ===================
float currentTemp = 0.0;           // Текущая температура (°C)
float desiredTemp = DEFAULT_TEMP;  // Желаемая температура (°C)
float targetTemp = DEFAULT_TEMP;   // Целевая температура для нагрева (°C)
bool heaterEnabled = false;        // Флаг включения нагрева
bool systemOn = false;             // Флаг включения системы
bool displayOn = false;            // Флаг включения дисплея - ИЗМЕНЕНО: по умолчанию выключен
bool displayNeedsUpdate = false;   // Флаг необходимости обновления дисплея

// Новые флаги для отображения температуры при достижении цели
bool targetReachedDisplay = false;     // Флаг отображения достижения цели
unsigned long targetReachedTime = 0;   // Время достижения цели
bool wasHeating = false;               // Флаг, что нагрев был активен

// Режимы работы
enum HeatingMode {
  MODE_IDLE,        // Ожидание
  MODE_HEATING,     // Нагрев (🔥)
  MODE_COOLING,     // Охлаждение (❄️)
  MODE_MAINTAIN     // Поддержание (=)
};

HeatingMode currentMode = MODE_IDLE;

// Переменные для СУПЕР плавного управления нагревом
float heatingPower = 0.0;          // Текущая мощность нагрева (%)
float lastHeatingPower = 0.0;      // Предыдущая мощность нагрева (%)
unsigned long lastHeatingUpdate = 0; // Время последнего обновления нагрева
bool overshootDetected = false;    // Флаг обнаружения перегрева
float overshootAmount = 0.0;       // Величина перегрева
float heatingIntegral = 0.0;       // Интегральная составляющая для плавности
float prevTempError = 0.0;         // Предыдущая ошибки температуры

// Фильтр для температуры
float tempHistory[5] = {0, 0, 0, 0, 0};
int tempHistoryIndex = 0;

// =================== ПЕРЕМЕННЫЕ БАТАРЕИ ===================
float batteryVoltage = 0.0;        // Напряжение батареи (В)
int batteryPercentage = 100;       // Процент заряда батареи (%)
unsigned long lastBatteryCheck = 0; // Время последней проверки батареи

// =================== ПЕРЕМЕННЫЕ КНОПОК ===================
unsigned long buttonPowerPressTime = 0;   // Время нажатия кнопки POWER
unsigned long buttonPowerReleaseTime = 0; // Время отпускания кнопки POWER
int buttonPowerClickCount = 0;            // Счетчик кликов кнопки POWER
bool buttonPowerPressed = false;          // Флаг нажатия кнопки POWER
bool buttonPowerLongPress = false;        // Флаг длительного нажатия
bool lastButtonState = HIGH;              // Предыдущее состояние кнопки

// =================== ПЕРЕМЕННЫЕ ВРЕМЕНИ ===================
unsigned long lastTempUpdate = 0;        // Время последнего обновления температуры
unsigned long lastDisplayUpdate = 0;     // Время последнего обновления дисплея
unsigned long lastActivity = 0;          // Время последней активности пользователя

void setup() {
  Serial.begin(115200);
  
  // =================== ОТКЛЮЧЕНИЕ Wi-Fi и Bluetooth ===================
  WiFi.mode(WIFI_OFF);                    // Выключаем Wi-Fi
  btStop();                               // Выключаем Bluetooth
  esp_bt_controller_disable();            // Полное отключение контроллера BT
  
  // Для ESP32-S3/S2/C3 может потребоваться:
  // esp_bluedroid_disable();
  // esp_bt_controller_deinit();
  
  Serial.println("Wi-Fi and Bluetooth disabled for power saving");
  Serial.println("Power saving mode: ACTIVE");
  // ===================================================================
  
  // Остальной код setup()...
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  
  // Настройка пинов
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  
  digitalWrite(HEATER_PIN, LOW);
  
  // Настройка ШИМ
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);
  
  // Инициализация дисплея
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  displayOn = false; // ДИСПЛЕЙ ВЫКЛЮЧЕН ПРИ СТАРТЕ
  
  // Инициализация EEPROM
  EEPROM.begin(128);
  
  // Загрузка температуры
  loadSavedTemperature();
  targetTemp = desiredTemp;
  
  // Тест термопары
  delay(500);
  float testTemp = thermocouple.readCelsius();
  if(isnan(testTemp) || testTemp < -50 || testTemp > 1000) {
    Serial.println("ERROR: Thermocouple not working!");
    displayErrorMessage("SENSOR ERROR");
  } else {
    currentTemp = testTemp;
    for (int i = 0; i < 5; i++) {
      tempHistory[i] = currentTemp;
    }
  }
  
  // Измерение батареи
  updateBatteryStatus();
  
  lastActivity = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Обработка кнопки POWER
  handlePowerButton();
  
  // Обработка кнопок температуры
  if (systemOn) {
    handleTemperatureButtons();
  }
  
  // Обновление температуры
  if (currentTime - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
    updateTemperature();
    
    // Определение режима работы с АДАПТИВНЫМИ ПОРОГАМИ
    if (heaterEnabled) {
      float tempDiff = targetTemp - currentTemp;
      float absTempDiff = abs(tempDiff);
      
      // АДАПТИВНЫЕ ПОРОГИ
      float adaptiveOvershootThreshold = max(3.0f, absTempDiff * 0.05f);
      float adaptiveMaintainThreshold = max(1.5f, absTempDiff * 0.02f); // Уменьшено до 2%
      
      // Проверка на перегрев
      if (currentTemp > targetTemp + adaptiveOvershootThreshold) {
        overshootDetected = true;
        overshootAmount = currentTemp - targetTemp;
        currentMode = MODE_COOLING;
        Serial.print("OVERSHOOT! +");
        Serial.print(overshootAmount, 1);
        Serial.println("°C");
      }
      // Если уже был перегрев
      else if (overshootDetected) {
        if (currentTemp <= targetTemp + 0.5f) { // Ждем полного остывания
          overshootDetected = false;
          currentMode = MODE_MAINTAIN;
          Serial.println("Overshoot cleared");
        } else {
          currentMode = MODE_COOLING;
        }
      }
      // Нормальная работа
      else {
        // Режим поддержания
        if (absTempDiff <= adaptiveMaintainThreshold) {
          currentMode = MODE_MAINTAIN;
          
          // Если только что достигли цели - включаем отображение на 5 секунд
          if (wasHeating && !targetReachedDisplay) {
            targetReachedDisplay = true;
            targetReachedTime = currentTime;
            displayOn = true; // ВКЛЮЧАЕМ ДИСПЛЕЙ
            Serial.println("Target temperature reached! Display ON for 5 sec");
          }
        }
        // Режим нагрева
        else if (tempDiff > 0) {
          currentMode = MODE_HEATING;
          wasHeating = true;
          targetReachedDisplay = false;
        }
        // Режим охлаждения (температура выше цели, но не перегрев)
        else {
          currentMode = MODE_COOLING;
        }
      }
    } else {
      // Нагреватель выключен - режим охлаждения
      currentMode = MODE_COOLING;
      overshootDetected = false;
      wasHeating = false;
    }
    
    lastTempUpdate = currentTime;
  }
  
  // Управление нагревом
  if (systemOn && heaterEnabled) {
    controlHeater();
  } else {
    ledcWrite(PWM_CHANNEL, 0);
    heatingPower = 0;
    lastHeatingPower = 0;
    overshootDetected = false;
    // Не меняем currentMode здесь - он уже установлен выше
  }
  
  // Обновление батареи
  if (currentTime - lastBatteryCheck >= BATTERY_UPDATE_INTERVAL) {
    updateBatteryStatus();
    lastBatteryCheck = currentTime;
  }
  
  // Обновление дисплея
  if (displayOn && systemOn) {
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      updateDisplay();
      lastDisplayUpdate = currentTime;
    }
  }
  
  // Таймаут дисплея (если не достигли цели)
  if (!targetReachedDisplay && systemOn && displayOn && (currentTime - lastActivity >= DISPLAY_TIMEOUT)) {
    displayOn = false;
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    Serial.println("Display timeout - OFF");
  }
  
  // Таймаут отображения достижения цели
  if (targetReachedDisplay && (currentTime - targetReachedTime >= DISPLAY_TARGET_REACHED_TIME)) {
    targetReachedDisplay = false;
    displayOn = false;
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    Serial.println("Target reached display timeout - OFF");
  }
  
  // Проверки безопасности
  safetyCheck();
}

// =================== ОБРАБОТКА КНОПКИ POWER ===================
void handlePowerButton() {
  unsigned long currentTime = millis();
  
  // Считываем состояние кнопки
  bool buttonState = digitalRead(BUTTON_POWER);
  
  // Обнаружение нажатия (переход из HIGH в LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Защита от дребезга
    if (currentTime - buttonPowerPressTime > 50) {
      buttonPowerPressed = true;
      buttonPowerPressTime = currentTime;
      lastActivity = currentTime;
    }
  }
  
  // Обнаружение отпускания (переход из LOW в HIGH)
  if (buttonState == HIGH && lastButtonState == LOW) {
    if (buttonPowerPressed) {
      buttonPowerPressed = false;
      buttonPowerReleaseTime = currentTime;
      lastActivity = currentTime;
      
      // Проверяем, было ли это длительное нажатие
      unsigned long pressDuration = currentTime - buttonPowerPressTime;
      
      // ДЛИТЕЛЬНОЕ НАЖАТИЕ (2 секунды) - вкл/выкл системы
      if (pressDuration >= BUTTON_HOLD_TIME) {
        buttonPowerLongPress = true;
        
        systemOn = !systemOn;
        displayOn = systemOn; // Включаем дисплей только если включаем систему
        lastActivity = currentTime;
        
        if (systemOn) {
          Serial.println("=== SYSTEM ON ===");
          heaterEnabled = false;
          currentMode = MODE_IDLE;
          overshootDetected = false;
          ledcWrite(PWM_CHANNEL, 0);
          heatingPower = 0;
          lastHeatingPower = 0;
          targetTemp = desiredTemp;
          wasHeating = false;
          targetReachedDisplay = false;
        } else {
          Serial.println("=== SYSTEM OFF ===");
          heaterEnabled = false;
          ledcWrite(PWM_CHANNEL, 0);
          heatingPower = 0;
          lastHeatingPower = 0;
          saveTemperatureToEEPROM();
        }
        
        updateDisplay();
        delay(300);
        buttonPowerClickCount = 0;
        return;
      }
      
      // КОРОТКОЕ НАЖАТИЕ
      else {
        buttonPowerLongPress = false;
        
        // Если дисплей выключен - ВКЛЮЧАЕМ ЕГО
        if (!displayOn) {
          displayOn = true;
          updateDisplay();
          buttonPowerClickCount = 0; // Сбрасываем счетчик кликов
          return;
        }
        
        // Если дисплей уже включен - считаем клики
        buttonPowerClickCount++;
        
        // ДВОЙНОЕ НАЖАТИЕ - вкл/выкл нагрева (только если дисплей ВКЛЮЧЕН)
        if (buttonPowerClickCount == 2 && displayOn) {
          if (systemOn) {
            if (!heaterEnabled && (batteryPercentage <= 0 || batteryVoltage < MIN_VOLTAGE_FOR_HEATING)) {
              updateDisplay();
              buttonPowerClickCount = 0;
              return;
            }
            
            heaterEnabled = !heaterEnabled;
            overshootDetected = false;
            heatingIntegral = 0.0;
            prevTempError = 0.0;
            lastActivity = currentTime;
            
            if (heaterEnabled) {
              Serial.println("=== HEATING ENABLED ===");
              Serial.print("Target temperature: ");
              Serial.println(desiredTemp);
              targetTemp = desiredTemp;
              heatingPower = HEATER_POWER_MIN;
              lastHeatingPower = HEATER_POWER_MIN;
              wasHeating = false;
              targetReachedDisplay = false;
              
              // Включаем дисплей при старте нагрева
              displayOn = true;
            } else {
              Serial.println("=== HEATING DISABLED ===");
              ledcWrite(PWM_CHANNEL, 0);
              heatingPower = 0;
              lastHeatingPower = 0;
              currentMode = MODE_IDLE;
              wasHeating = false;
            }
            
            updateDisplay();
          }
          buttonPowerClickCount = 0;
        }
      }
    }
  }
  
  // Сохраняем состояние кнопки
  lastButtonState = buttonState;
  
  // Сброс счетчика кликов через 400 мс
  if (buttonPowerClickCount > 0 && (currentTime - buttonPowerPressTime >= 400)) {
    buttonPowerClickCount = 0;
  }
}

// =================== ОБРАБОТКА КНОПОК ТЕМПЕРАТУРЫ ===================
void handleTemperatureButtons() {
  unsigned long currentTime = millis();
  static unsigned long lastButtonTime = 0;
  
  if (currentTime - lastButtonTime < BUTTON_DEBOUNCE) {
    return;
  }
  
  bool tempChanged = false;
  
  if (digitalRead(BUTTON_UP) == LOW) {
    lastButtonTime = currentTime;
    lastActivity = currentTime;
    
    desiredTemp += TEMP_STEP;
    if (desiredTemp > TEMP_MAX) desiredTemp = TEMP_MAX;
    tempChanged = true;
  }
  
  if (digitalRead(BUTTON_DOWN) == LOW) {
    lastButtonTime = currentTime;
    lastActivity = currentTime;
    
    desiredTemp -= TEMP_STEP;
    if (desiredTemp < TEMP_MIN) desiredTemp = TEMP_MIN;
    tempChanged = true;
  }
  
  if (tempChanged) {
    if (heaterEnabled) {
      targetTemp = desiredTemp;
      overshootDetected = false;
      heatingIntegral = 0.0;
      prevTempError = 0.0;
      wasHeating = false;
      targetReachedDisplay = false;
      Serial.print("Target changed to: ");
      Serial.println(desiredTemp);
    }
    
    saveTemperatureToEEPROM();
    displayOn = true; // Включаем дисплей при изменении температуры
    updateDisplay();
    delay(BUTTON_DEBOUNCE);
  }
}

void controlHeater() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastHeatingUpdate < HEATING_UPDATE_INTERVAL) {
    return;
  }
  lastHeatingUpdate = currentTime;
  
  // Проверка безопасности
  if (currentTemp >= MAX_TEMPERATURE) {
    heaterEnabled = false;
    ledcWrite(PWM_CHANNEL, 0);
    heatingPower = 0;
    lastHeatingPower = 0;
    currentMode = MODE_IDLE;
    Serial.print("SAFETY: Overheat at ");
    Serial.print(currentTemp, 1);
    Serial.println("°C");
    return;
  }
  
  // Проверка батареи
  if (batteryPercentage <= 0 || batteryVoltage < MIN_VOLTAGE_FOR_HEATING) {
    heaterEnabled = false;
    ledcWrite(PWM_CHANNEL, 0);
    heatingPower = 0;
    lastHeatingPower = 0;
    currentMode = MODE_IDLE;
    Serial.println("Heater disabled: battery too low!");
    return;
  }
  
  // Расчет разницы температур
  float tempDiff = targetTemp - currentTemp;
  float absTempDiff = abs(tempDiff);
  
  // =================== АДАПТИВНАЯ ЛОГИКА УПРАВЛЕНИЯ ===================
  // Чем больше разница - тем осторожнее нагреваем!
  
  // Адаптивная максимальная мощность
  float adaptiveMaxPower = HEATER_POWER_MAX;
  if (absTempDiff > 100.0f) {
    adaptiveMaxPower = HEATER_POWER_MAX * 0.6f; // 40% меньше при +100°C
  } else if (absTempDiff > 50.0f) {
    adaptiveMaxPower = HEATER_POWER_MAX * 0.7f; // 30% меньше при +50°C
  } else if (absTempDiff > 20.0f) {
    adaptiveMaxPower = HEATER_POWER_MAX * 0.8f; // 20% меньше при +20°C
  }
  
  // Адаптивная скорость изменения мощности
  float adaptiveMaxChange = 1.0f;
  if (absTempDiff > 50.0f) {
    adaptiveMaxChange = 0.7f; // Медленнее при больших скачках
  }
  
  float newPower = 0.0;
  
  switch (currentMode) {
    case MODE_MAINTAIN:
      // РЕЖИМ ПОДДЕРЖАНИЯ (=)
      if (currentTemp < targetTemp - 1.0) {
        newPower = HEATER_POWER_MAINTAIN;
      } else if (currentTemp > targetTemp + 0.5) {
        newPower = 0;
      } else {
        newPower = HEATER_POWER_MAINTAIN / 3;
      }
      break;
      
    case MODE_HEATING:
      // РЕЖИМ НАГРЕВА (🔥) - АДАПТИВНЫЙ
      if (tempDiff > 100.0f) {
        newPower = 25.0f;
      } 
      else if (tempDiff > 50.0f) {
        newPower = 30.0f;
      }
      else if (tempDiff > 20.0f) {
        newPower = 35.0f;
      }
      else if (tempDiff > 10.0f) {
        newPower = 25.0f;
      }
      else if (tempDiff > 5.0f) {
        newPower = 15.0f;
      }
      else if (tempDiff > 2.0f) {
        newPower = 8.0f;
      }
      else {
        newPower = HEATER_POWER_MIN;
      }
      
      // Сильное снижение мощности при приближении к цели
      if (tempDiff < 20.0f) {
        float factor = tempDiff / 20.0f;
        newPower = newPower * factor * 0.8f; // Дополнительное снижение
        if (newPower < HEATER_POWER_MIN) {
          newPower = HEATER_POWER_MIN;
        }
      }
      
      // Ограничение адаптивной мощностью
      if (newPower > adaptiveMaxPower) {
        newPower = adaptiveMaxPower;
      }
      break;
      
    case MODE_COOLING:
      // РЕЖИМ ОХЛАЖДЕНИЯ (❄️)
      newPower = 0;
      break;
      
    case MODE_IDLE:
    default:
      newPower = 0;
      break;
  }
  
  // Плавное изменение мощности с адаптивной скоростью
  float powerChange = newPower - lastHeatingPower;
  
  if (abs(powerChange) > adaptiveMaxChange) {
    if (powerChange > 0) {
      lastHeatingPower += adaptiveMaxChange;
    } else {
      lastHeatingPower -= adaptiveMaxChange;
    }
  } else {
    lastHeatingPower = newPower;
  }
  
  lastHeatingPower = constrain(lastHeatingPower, 0, adaptiveMaxPower);
  
  // Применение ШИМ
  int pwmValue = map(lastHeatingPower, 0, 100, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);
  ledcWrite(PWM_CHANNEL, pwmValue);
  heatingPower = lastHeatingPower;
  
  // Отладочный вывод
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug > 2000) {
    const char* modeStr = "";
    switch (currentMode) {
      case MODE_HEATING: modeStr = "HEATING"; break;
      case MODE_COOLING: modeStr = "COOLING"; break;
      case MODE_MAINTAIN: modeStr = "MAINTAIN"; break;
      default: modeStr = "IDLE"; break;
    }
    
    Serial.print("Mode: ");
    Serial.print(modeStr);
    Serial.print(", Temp: ");
    Serial.print(currentTemp, 1);
    Serial.print("°C / ");
    Serial.print(targetTemp);
    Serial.print("°C, Diff: ");
    Serial.print(tempDiff, 1);
    Serial.print("°C, Power: ");
    Serial.print(heatingPower, 1);
    Serial.print("%, AdaptiveMax: ");
    Serial.print(adaptiveMaxPower, 0);
    Serial.print("%, ChangeRate: ");
    Serial.print(adaptiveMaxChange, 1);
    Serial.print("%/step");
    Serial.println();
    lastDebug = currentTime;
  }
}

// =================== ОБНОВЛЕНИЕ ТЕМПЕРАТУРЫ ===================
Filter your search...
Type:

All





void updateTemperature() {
  float newTemp = thermocouple.readCelsius();
  
  if (isnan(newTemp) || newTemp < -50 || newTemp > 1000) {
    return;
  }
  
  // Медианный фильтр
  tempHistory[tempHistoryIndex] = newTemp;
  tempHistoryIndex = (tempHistoryIndex + 1) % 5;
  
  float tempArray[5];
  for (int i = 0; i < 5; i++) {
    tempArray[i] = tempHistory[i];
  }
  
  // Сортировка для медианы
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (tempArray[j] < tempArray[i]) {
        float temp = tempArray[i];
        tempArray[i] = tempArray[j];
        tempArray[j] = temp;
      }
    }
  }
  
  float medianTemp = tempArray[2];
  
  // Экспоненциальное сглаживание (более быстрое реагирование)
  float alpha = 0.7;
  currentTemp = currentTemp * alpha + medianTemp * (1.0 - alpha);
}

void updateDisplay() {
  if (!displayOn) return;
  
  u8g2.clearBuffer();
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 128, 64);
  u8g2.setDrawColor(1);
  
  // ПРОСТОЙ РЕЖИМ - только большая температура
  u8g2.setFont(u8g2_font_logisoso38_tn);
  char tempStr[6];
  sprintf(tempStr, "%d", (int)currentTemp);
  int tempWidth = u8g2.getStrWidth(tempStr);
  int tempX = (128 - tempWidth) / 2;
  int tempY = 45;
  u8g2.drawStr(tempX, tempY, tempStr);
  
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(tempX + tempWidth + 5, 30, "o");
  u8g2.drawStr(tempX + tempWidth + 11, 30, "C");
  
  // СИМВОЛ РЕЖИМА - показываем ТОЛЬКО когда нагрев ВКЛЮЧЕН
  int symbolX = tempX + tempWidth + 15;
  if (heaterEnabled) {
    switch (currentMode) {
      case MODE_HEATING:
        drawHeatingSymbol(symbolX, tempY + 10);  // 🔥
        break;
      case MODE_COOLING:
        drawCoolingSymbol(symbolX, tempY + 10);  // ❄️
        break;
      case MODE_MAINTAIN:
        drawMaintainSymbol(symbolX, tempY + 10); // =
        break;
      default:
        drawCoolingSymbol(symbolX, tempY + 10);  // ❄️ по умолчанию
        break;
    }
  }
  // Если нагрев выключен - не показываем символ
  
  u8g2.sendBuffer();
}

// =================== ДОПОЛНИТЕЛЬНЫЕ ФУНКЦИИ ===================

void drawDegree(int x, int y) {
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
}

void drawHeatingSymbol(int x, int y) {
  // Символ пламени (🔥)
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

void drawMaintainSymbol(int x, int y) {
  // Символ "=" (поддержание)
  u8g2.drawLine(x, y + 2, x + 8, y + 2);
  u8g2.drawLine(x, y + 6, x + 8, y + 6);
}

void drawCoolingSymbol(int x, int y) {
  // Символ снежинки (❄️)
  u8g2.drawLine(x, y - 4, x, y + 4);
  u8g2.drawLine(x - 4, y, x + 4, y);
  u8g2.drawLine(x - 3, y - 3, x + 3, y + 3);
  u8g2.drawLine(x - 3, y + 3, x + 3, y - 3);
  u8g2.drawPixel(x, y);
}

void drawBatteryIndicator(int x, int y, int percentage) {
  u8g2.drawFrame(x, y, 20, 10);
  u8g2.drawBox(x + 20, y + 3, 2, 4);
  
  int fillWidth = map(percentage, 0, 100, 0, 18);
  fillWidth = constrain(fillWidth, 0, 18);
  
  u8g2.drawBox(x + 1, y + 1, fillWidth, 8);
}

void updateBatteryStatus() {
  const int numSamples = 10;
  long sum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(100);
  }
  
  int rawValue = sum / numSamples;
  
  if (rawValue < 10) return;
  
  float adcVoltage = (rawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  batteryVoltage = adcVoltage * VOLTAGE_DIVIDER_COEFF;
  
  // Улучшенный расчет процентов с линейной интерполяцией
  if (batteryVoltage >= BATTERY_FULL_VOLTAGE) {
    batteryPercentage = 100;
  } else if (batteryVoltage <= BATTERY_EMPTY_VOLTAGE) {
    batteryPercentage = 0;
  } else {
    // Линейная интерполяция между пустым и полным
    float voltageRange = BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE;
    float voltageDiff = batteryVoltage - BATTERY_EMPTY_VOLTAGE;
    batteryPercentage = (int)((voltageDiff / voltageRange) * 100.0f);
  }
  
  // Отладочный вывод для калибровки
  static unsigned long lastBatteryDebug = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastBatteryDebug > 10000) {
    Serial.print("Battery calibration - RAW ADC: ");
    Serial.print(rawValue);
    Serial.print(", ADC Voltage: ");
    Serial.print(adcVoltage, 3);
    Serial.print("V, Battery Voltage: ");
    Serial.print(batteryVoltage, 3);
    Serial.print("V, Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("%");
    
    // Подсказка для калибровки
    Serial.println("=== CALIBRATION TIP ===");
    Serial.println("If battery always shows 100%, try these VOLTAGE_DIVIDER_COEFF:");
    Serial.println("- For 1:2 divider (4.2V->2.1V): 2.0");
    Serial.println("- For 1:3 divider (4.2V->1.4V): 3.0");
    Serial.print("Current coefficient: ");
    Serial.println(VOLTAGE_DIVIDER_COEFF);
    Serial.println("Measure actual battery voltage and adjust coefficient!");
    
    lastBatteryDebug = currentTime;
  }
}

void safetyCheck() {
  if (currentTemp >= MAX_TEMPERATURE && heaterEnabled) {
    heaterEnabled = false;
    ledcWrite(PWM_CHANNEL, 0);
    currentMode = MODE_IDLE;
  }
}

void saveTemperatureToEEPROM() {
  int tempToSave = (int)desiredTemp;
  if (tempToSave > 255) tempToSave = 255;
  EEPROM.write(0, tempToSave);
  EEPROM.commit();
}

void loadSavedTemperature() {
  int savedValue = EEPROM.read(0);
  if (savedValue >= TEMP_MIN && savedValue <= TEMP_MAX) {
    desiredTemp = savedValue;
  } else {
    desiredTemp = DEFAULT_TEMP;
  }
}

void displayErrorMessage(const char* message) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(10, 30, message);
  u8g2.sendBuffer();
  delay(3000);
}