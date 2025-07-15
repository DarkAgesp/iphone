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
#define MOSFET_GATE_PIN D5      // Управление затвором MOSFET
#define POTENTIOMETER_PIN A0    // Потенциометр (можно использовать мультиплексор)
#define LED_PIN D4              // Встроенный светодиод для индикации

// Constants
#define SHUNT_RESISTANCE 0.1    // Сопротивление шунта в омах
#define MAX_CURRENT 1.0         // Максимальный ток в амперах
#define VOLTAGE_DIVIDER_RATIO 0.164 // (10k / (10k + 51k)) = 0.164
#define ADC_MAX_VALUE 1024      // Максимальное значение АЦП
#define REFERENCE_VOLTAGE 3.3   // Опорное напряжение ESP8266
#define SAMPLES_COUNT 10        // Количество выборок для усреднения

// Variables
float currentCurrent = 0.0;
float currentVoltage = 0.0;
float shuntVoltage = 0.0;
bool powerEnabled = false;
float powerHistory[SCREEN_WIDTH];
int historyIndex = 0;
unsigned long lastUpdate = 0;
unsigned long lastGraphUpdate = 0;
const unsigned long UPDATE_INTERVAL = 100;  // Обновление каждые 100мс
const unsigned long GRAPH_UPDATE_INTERVAL = 500; // Обновление графика каждые 500мс

void setup() {
  Serial.begin(115200);
  Serial.println("iPhone Power Monitor Starting...");
  
  // Initialize pins
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN, LOW); // Выключить питание по умолчанию
  digitalWrite(LED_PIN, HIGH); // Выключить светодиод (инвертированная логика)
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Остановить выполнение
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("iPhone Power Monitor");
  display.println("Initializing...");
  display.display();
  
  // Initialize power history array
  for(int i = 0; i < SCREEN_WIDTH; i++) {
    powerHistory[i] = 0.0;
  }
  
  delay(2000);
  Serial.println("Setup complete");
}

float readAverageVoltage(int pin, int samples) {
  float sum = 0.0;
  for(int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return (sum / samples) * (REFERENCE_VOLTAGE / ADC_MAX_VALUE);
}

void updateMeasurements() {
  // Читаем напряжение на шунте
  shuntVoltage = readAverageVoltage(SHUNT_VOLTAGE_PIN, SAMPLES_COUNT);
  
  // Вычисляем ток через шунт (I = U / R)
  currentCurrent = shuntVoltage / SHUNT_RESISTANCE;
  
  // Вычисляем напряжение питания (с учетом делителя напряжения)
  currentVoltage = shuntVoltage / VOLTAGE_DIVIDER_RATIO;
  
  // Защита от превышения тока
  if(currentCurrent > MAX_CURRENT) {
    powerEnabled = false;
    digitalWrite(MOSFET_GATE_PIN, LOW);
    digitalWrite(LED_PIN, HIGH); // Выключить светодиод
    Serial.println("OVERCURRENT PROTECTION TRIGGERED!");
  }
}

void updatePowerHistory() {
  float power = currentCurrent * currentVoltage;
  powerHistory[historyIndex] = power;
  historyIndex = (historyIndex + 1) % SCREEN_WIDTH;
}

void drawGraph() {
  // Очистить область графика
  display.fillRect(0, 32, SCREEN_WIDTH, 32, SSD1306_BLACK);
  
  // Найти максимальное значение для масштабирования
  float maxPower = 0.1; // Минимальный масштаб
  for(int i = 0; i < SCREEN_WIDTH; i++) {
    if(powerHistory[i] > maxPower) {
      maxPower = powerHistory[i];
    }
  }
  
  // Нарисовать график
  for(int x = 1; x < SCREEN_WIDTH; x++) {
    int prevIndex = (historyIndex + x - 1) % SCREEN_WIDTH;
    int currIndex = (historyIndex + x) % SCREEN_WIDTH;
    
    int y1 = 63 - (int)((powerHistory[prevIndex] / maxPower) * 31);
    int y2 = 63 - (int)((powerHistory[currIndex] / maxPower) * 31);
    
    display.drawLine(x-1, y1, x, y2, SSD1306_WHITE);
  }
  
  // Показать масштаб
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(maxPower, 2);
  display.print("W");
}

void updateDisplay() {
  display.clearDisplay();
  
  // Заголовок
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("iPhone Power Monitor");
  
  // Статус питания
  display.setCursor(0, 10);
  display.print("Power: ");
  display.print(powerEnabled ? "ON" : "OFF");
  
  // Ток
  display.setCursor(0, 20);
  display.print("I: ");
  display.print(currentCurrent, 3);
  display.print("A");
  
  // Напряжение
  display.setCursor(70, 20);
  display.print("U: ");
  display.print(currentVoltage, 2);
  display.print("V");
  
  // Мощность
  display.setCursor(0, 30);
  display.print("P: ");
  display.print(currentCurrent * currentVoltage, 3);
  display.print("W");
  
  // График потребления
  drawGraph();
  
  display.display();
}

void handleSerialCommands() {
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command == "ON") {
      powerEnabled = true;
      digitalWrite(MOSFET_GATE_PIN, HIGH);
      digitalWrite(LED_PIN, LOW); // Включить светодиод
      Serial.println("Power enabled");
    }
    else if(command == "OFF") {
      powerEnabled = false;
      digitalWrite(MOSFET_GATE_PIN, LOW);
      digitalWrite(LED_PIN, HIGH); // Выключить светодиод
      Serial.println("Power disabled");
    }
    else if(command == "STATUS") {
      Serial.print("Power: ");
      Serial.println(powerEnabled ? "ON" : "OFF");
      Serial.print("Current: ");
      Serial.print(currentCurrent, 3);
      Serial.println("A");
      Serial.print("Voltage: ");
      Serial.print(currentVoltage, 2);
      Serial.println("V");
      Serial.print("Power: ");
      Serial.print(currentCurrent * currentVoltage, 3);
      Serial.println("W");
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // Обновление измерений
  if(currentTime - lastUpdate >= UPDATE_INTERVAL) {
    updateMeasurements();
    updateDisplay();
    lastUpdate = currentTime;
    
    // Отладочный вывод в Serial
    Serial.print("I: ");
    Serial.print(currentCurrent, 3);
    Serial.print("A, U: ");
    Serial.print(currentVoltage, 2);
    Serial.print("V, P: ");
    Serial.print(currentCurrent * currentVoltage, 3);
    Serial.println("W");
  }
  
  // Обновление графика
  if(currentTime - lastGraphUpdate >= GRAPH_UPDATE_INTERVAL) {
    updatePowerHistory();
    lastGraphUpdate = currentTime;
  }
  
  // Обработка команд из Serial
  handleSerialCommands();
  
  delay(10);
}