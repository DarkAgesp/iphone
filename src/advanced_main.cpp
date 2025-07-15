#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
#define SHUNT_VOLTAGE_PIN A0    // Аналоговый вход для чтения напряжения на шунте
#define MOSFET_GATE_PIN D5      // Управление затвором MOSFET (PWM)
#define BUTTON_PIN D3           // Кнопка для переключения режимов
#define LED_PIN D4              // Встроенный светодиод для индикации

// Constants
#define SHUNT_RESISTANCE 0.1    // Сопротивление шунта в омах
#define MAX_CURRENT 1.0         // Максимальный ток в амперах
#define VOLTAGE_DIVIDER_RATIO 0.164 // (10k / (10k + 51k)) = 0.164
#define ADC_MAX_VALUE 1024      // Максимальное значение АЦП
#define REFERENCE_VOLTAGE 3.3   // Опорное напряжение ESP8266
#define SAMPLES_COUNT 10        // Количество выборок для усреднения
#define PWM_MAX 1023           // Максимальное значение PWM

// Variables
float currentCurrent = 0.0;
float currentVoltage = 0.0;
float shuntVoltage = 0.0;
float targetCurrent = 0.5;     // Целевой ток (управляется потенциометром)
int pwmValue = 0;              // Значение PWM для управления MOSFET
bool powerEnabled = false;
bool autoMode = false;         // Автоматический режим управления током
float powerHistory[SCREEN_WIDTH];
int historyIndex = 0;
unsigned long lastUpdate = 0;
unsigned long lastGraphUpdate = 0;
unsigned long lastButtonPress = 0;
const unsigned long UPDATE_INTERVAL = 100;
const unsigned long GRAPH_UPDATE_INTERVAL = 500;
const unsigned long DEBOUNCE_DELAY = 200;

// PID Controller variables
float pidKp = 100.0;           // Пропорциональный коэффициент
float pidKi = 10.0;            // Интегральный коэффициент  
float pidKd = 1.0;             // Дифференциальный коэффициент
float pidIntegral = 0.0;
float pidPrevError = 0.0;
unsigned long pidLastTime = 0;

enum DisplayMode {
  MODE_MAIN,
  MODE_GRAPH,
  MODE_SETTINGS
};

DisplayMode currentMode = MODE_MAIN;

void setup() {
  Serial.begin(115200);
  Serial.println("iPhone Power Monitor Advanced Starting...");
  
  // Initialize pins
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  analogWrite(MOSFET_GATE_PIN, 0); // Выключить питание по умолчанию
  digitalWrite(LED_PIN, HIGH);     // Выключить светодиод
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("iPhone Power Monitor");
  display.println("Advanced Version");
  display.println("Initializing...");
  display.display();
  
  // Initialize power history array
  for(int i = 0; i < SCREEN_WIDTH; i++) {
    powerHistory[i] = 0.0;
  }
  
  delay(2000);
  Serial.println("Setup complete");
  Serial.println("Commands: ON, OFF, AUTO, MANUAL, SET_TARGET <value>, STATUS");
}

float readAverageVoltage(int pin, int samples) {
  float sum = 0.0;
  for(int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return (sum / samples) * (REFERENCE_VOLTAGE / ADC_MAX_VALUE);
}

float pidController(float setpoint, float input) {
  unsigned long now = millis();
  float timeChange = (float)(now - pidLastTime) / 1000.0; // в секундах
  
  if(timeChange >= 0.1) { // Обновляем PID каждые 100мс
    float error = setpoint - input;
    
    pidIntegral += error * timeChange;
    // Ограничиваем интегральную составляющую
    if(pidIntegral > PWM_MAX/pidKi) pidIntegral = PWM_MAX/pidKi;
    if(pidIntegral < -PWM_MAX/pidKi) pidIntegral = -PWM_MAX/pidKi;
    
    float derivative = (error - pidPrevError) / timeChange;
    
    float output = pidKp * error + pidKi * pidIntegral + pidKd * derivative;
    
    pidPrevError = error;
    pidLastTime = now;
    
    // Ограничиваем выход
    if(output > PWM_MAX) output = PWM_MAX;
    if(output < 0) output = 0;
    
    return output;
  }
  
  return pwmValue; // Возвращаем предыдущее значение если время не пришло
}

void updateMeasurements() {
  // Читаем напряжение на шунте
  shuntVoltage = readAverageVoltage(SHUNT_VOLTAGE_PIN, SAMPLES_COUNT);
  
  // Вычисляем ток через шунт
  currentCurrent = shuntVoltage / SHUNT_RESISTANCE;
  
  // Вычисляем напряжение питания
  currentVoltage = shuntVoltage / VOLTAGE_DIVIDER_RATIO;
  
  // Автоматическое управление током
  if(autoMode && powerEnabled) {
    pwmValue = (int)pidController(targetCurrent, currentCurrent);
    analogWrite(MOSFET_GATE_PIN, pwmValue);
  }
  
  // Защита от превышения тока
  if(currentCurrent > MAX_CURRENT) {
    powerEnabled = false;
    autoMode = false;
    pwmValue = 0;
    analogWrite(MOSFET_GATE_PIN, 0);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("OVERCURRENT PROTECTION TRIGGERED!");
  }
  
  // Управление светодиодом
  if(powerEnabled) {
    // Мигание с частотой пропорциональной току
    int blinkRate = (int)(currentCurrent * 1000);
    digitalWrite(LED_PIN, (millis() % max(100, 1000 - blinkRate)) < 50 ? LOW : HIGH);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}

void updatePowerHistory() {
  float power = currentCurrent * currentVoltage;
  powerHistory[historyIndex] = power;
  historyIndex = (historyIndex + 1) % SCREEN_WIDTH;
}

void drawMainScreen() {
  display.clearDisplay();
  
  // Заголовок
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("iPhone Monitor ");
  display.print(autoMode ? "AUTO" : "MAN");
  
  // Статус питания
  display.setCursor(0, 10);
  display.print("Power: ");
  display.print(powerEnabled ? "ON" : "OFF");
  
  if(autoMode) {
    display.print(" T:");
    display.print(targetCurrent, 2);
    display.print("A");
  }
  
  // Ток
  display.setCursor(0, 20);
  display.print("Current: ");
  display.print(currentCurrent, 3);
  display.print("A");
  
  // Напряжение
  display.setCursor(0, 30);
  display.print("Voltage: ");
  display.print(currentVoltage, 2);
  display.print("V");
  
  // Мощность
  display.setCursor(0, 40);
  display.print("Power: ");
  display.print(currentCurrent * currentVoltage, 3);
  display.print("W");
  
  // PWM значение
  display.setCursor(0, 50);
  display.print("PWM: ");
  display.print(pwmValue);
  display.print("/");
  display.print(PWM_MAX);
  
  display.display();
}

void drawGraph() {
  display.clearDisplay();
  
  // Заголовок
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Power Graph");
  
  // Найти максимальное значение для масштабирования
  float maxPower = 0.1;
  for(int i = 0; i < SCREEN_WIDTH; i++) {
    if(powerHistory[i] > maxPower) {
      maxPower = powerHistory[i];
    }
  }
  
  // Нарисовать график
  for(int x = 1; x < SCREEN_WIDTH; x++) {
    int prevIndex = (historyIndex + x - 1) % SCREEN_WIDTH;
    int currIndex = (historyIndex + x) % SCREEN_WIDTH;
    
    int y1 = 63 - (int)((powerHistory[prevIndex] / maxPower) * 45);
    int y2 = 63 - (int)((powerHistory[currIndex] / maxPower) * 45);
    
    if(y1 < 10) y1 = 10;
    if(y2 < 10) y2 = 10;
    
    display.drawLine(x-1, y1, x, y2, SSD1306_WHITE);
  }
  
  // Показать масштаб
  display.setCursor(0, 10);
  display.print("Max: ");
  display.print(maxPower, 2);
  display.print("W");
  
  display.display();
}

void drawSettings() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Settings");
  
  display.setCursor(0, 15);
  display.print("Target I: ");
  display.print(targetCurrent, 2);
  display.print("A");
  
  display.setCursor(0, 25);
  display.print("PID Kp: ");
  display.print(pidKp, 1);
  
  display.setCursor(0, 35);
  display.print("PID Ki: ");
  display.print(pidKi, 1);
  
  display.setCursor(0, 45);
  display.print("PID Kd: ");
  display.print(pidKd, 1);
  
  display.setCursor(0, 55);
  display.print("Mode: ");
  display.print(autoMode ? "AUTO" : "MANUAL");
  
  display.display();
}

void handleButton() {
  if(digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = millis();
    
    currentMode = (DisplayMode)((currentMode + 1) % 3);
    
    Serial.print("Mode changed to: ");
    switch(currentMode) {
      case MODE_MAIN: Serial.println("MAIN"); break;
      case MODE_GRAPH: Serial.println("GRAPH"); break;
      case MODE_SETTINGS: Serial.println("SETTINGS"); break;
    }
  }
}

void handleSerialCommands() {
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if(command == "ON") {
      powerEnabled = true;
      if(!autoMode) {
        pwmValue = PWM_MAX;
        analogWrite(MOSFET_GATE_PIN, pwmValue);
      }
      Serial.println("Power enabled");
    }
    else if(command == "OFF") {
      powerEnabled = false;
      autoMode = false;
      pwmValue = 0;
      analogWrite(MOSFET_GATE_PIN, 0);
      pidIntegral = 0; // Сброс интегральной составляющей
      Serial.println("Power disabled");
    }
    else if(command == "AUTO") {
      autoMode = true;
      powerEnabled = true;
      pidIntegral = 0;
      Serial.println("Auto mode enabled");
    }
    else if(command == "MANUAL") {
      autoMode = false;
      Serial.println("Manual mode enabled");
    }
    else if(command.startsWith("SET_TARGET ")) {
      float newTarget = command.substring(11).toFloat();
      if(newTarget > 0 && newTarget <= MAX_CURRENT) {
        targetCurrent = newTarget;
        pidIntegral = 0; // Сброс при изменении уставки
        Serial.print("Target current set to: ");
        Serial.print(targetCurrent, 3);
        Serial.println("A");
      } else {
        Serial.println("Invalid target current");
      }
    }
    else if(command.startsWith("SET_PWM ")) {
      if(!autoMode) {
        int newPwm = command.substring(8).toInt();
        if(newPwm >= 0 && newPwm <= PWM_MAX) {
          pwmValue = newPwm;
          if(powerEnabled) {
            analogWrite(MOSFET_GATE_PIN, pwmValue);
          }
          Serial.print("PWM set to: ");
          Serial.println(pwmValue);
        }
      } else {
        Serial.println("Cannot set PWM in auto mode");
      }
    }
    else if(command == "STATUS") {
      Serial.println("=== STATUS ===");
      Serial.print("Power: ");
      Serial.println(powerEnabled ? "ON" : "OFF");
      Serial.print("Mode: ");
      Serial.println(autoMode ? "AUTO" : "MANUAL");
      Serial.print("Current: ");
      Serial.print(currentCurrent, 3);
      Serial.println("A");
      Serial.print("Target: ");
      Serial.print(targetCurrent, 3);
      Serial.println("A");
      Serial.print("Voltage: ");
      Serial.print(currentVoltage, 2);
      Serial.println("V");
      Serial.print("Power: ");
      Serial.print(currentCurrent * currentVoltage, 3);
      Serial.println("W");
      Serial.print("PWM: ");
      Serial.print(pwmValue);
      Serial.print("/");
      Serial.println(PWM_MAX);
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // Обновление измерений
  if(currentTime - lastUpdate >= UPDATE_INTERVAL) {
    updateMeasurements();
    
    // Обновление дисплея в зависимости от режима
    switch(currentMode) {
      case MODE_MAIN:
        drawMainScreen();
        break;
      case MODE_GRAPH:
        drawGraph();
        break;
      case MODE_SETTINGS:
        drawSettings();
        break;
    }
    
    lastUpdate = currentTime;
  }
  
  // Обновление графика
  if(currentTime - lastGraphUpdate >= GRAPH_UPDATE_INTERVAL) {
    updatePowerHistory();
    lastGraphUpdate = currentTime;
  }
  
  // Обработка кнопки
  handleButton();
  
  // Обработка команд из Serial
  handleSerialCommands();
  
  delay(10);
}