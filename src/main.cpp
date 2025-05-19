#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Piny fizyczne
#define BUZZER_PIN      25    // Sygnał buzzera
#define LIGHT_SENSOR_PIN 32   // D0 z czujnika światła (cyfrowy)
#define HALL_SENSOR_PIN 33    // Czujnik Halla (cyfrowy z przerwaniem)
#define POWER_LED_PIN   26    // MOSFET do LED 1W
#define LEFT_LED1_PIN  14
#define LEFT_LED2_PIN  27
#define LEFT_LED3_PIN  12
#define RIGHT_LED1_PIN 15
#define RIGHT_LED2_PIN 2
#define RIGHT_LED3_PIN 4

// Kierunkowskazy
bool leftBlinking = false;
bool rightBlinking = false;
bool hazardMode = false;
unsigned long lastBlinkTime = 0;
bool blinkState = false; // HIGH/LOW migania
const unsigned long blinkInterval = 500; // ms

// BLE
BLECharacteristic *bleChar;
String bleCommand = "";
bool bleClientConnected = false;

// Akcelerometr
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
unsigned long tiltStartTime = 0;
bool isTilting = false;

// Progi
#define TILT_THRESHOLD   20
#define TILT_DURATION    5000
#define HALL_SAMPLE_NUM  5
#define WHEEL_CIRC_CM    210.0

bool awaitingAccidentConfirmation = false;
unsigned long accidentQueryStart = 0;
const unsigned long accidentResponseWindow = 10000; // 10 sekund

// Czujnik Halla
volatile int hallPulseCount = 0;
unsigned long hallLastTime = 0;

// BLE Callback
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxVal = pCharacteristic->getValue();
    bleCommand = String(rxVal.c_str());
    Serial.print("BLE Received: ");
    Serial.println(bleCommand);

    // Obsługa kierunkowskazów
  if (bleCommand == "LEFT") {
    leftBlinking = !leftBlinking;
    Serial.println(leftBlinking ? "LEFT ON" : "LEFT OFF");
  } else if (bleCommand == "RIGHT") {
    rightBlinking = !rightBlinking;
    Serial.println(rightBlinking ? "RIGHT ON" : "RIGHT OFF");
  } else if (bleCommand == "HAZARD") {
    leftBlinking = rightBlinking = !(leftBlinking || rightBlinking);
    Serial.println(leftBlinking ? "HAZARD ON" : "HAZARD OFF");
  }
  }
};

void setLEDs(int pin1, int pin2, int pin3, bool state) {
  digitalWrite(pin1, state);
  digitalWrite(pin2, state);
  digitalWrite(pin3, state);
}

void turnOffLeft() {
  setLEDs(LEFT_LED1_PIN, LEFT_LED2_PIN, LEFT_LED3_PIN, LOW);
}
void turnOffRight() {
  setLEDs(RIGHT_LED1_PIN, RIGHT_LED2_PIN, RIGHT_LED3_PIN, LOW);
}

// Przerwanie od czujnika Halla
void IRAM_ATTR hallInterrupt() {
  hallPulseCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  // Piny kierunkowskazów jako OUTPUT
  pinMode(LEFT_LED1_PIN, OUTPUT);
  pinMode(LEFT_LED2_PIN, OUTPUT);
  pinMode(LEFT_LED3_PIN, OUTPUT);
  pinMode(RIGHT_LED1_PIN, OUTPUT);
  pinMode(RIGHT_LED2_PIN, OUTPUT);
  pinMode(RIGHT_LED3_PIN, OUTPUT);

  turnOffLeft();
  turnOffRight();

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallInterrupt, RISING);

  if (!accel.begin()) {
    Serial.println("Brak ADXL345!");
    //while (1);
  }

  // BLE inicjalizacja
  BLEDevice::init("ESP32_Module");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789abc");
  bleChar = pService->createCharacteristic("abcd", BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  bleChar->addDescriptor(new BLE2902());
  bleChar->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("System uruchomiony.");
}

void loop() {
  // 💡 Odczyt światła i sterowanie Power LED
  bool isDark = digitalRead(LIGHT_SENSOR_PIN) == HIGH;
  digitalWrite(POWER_LED_PIN, isDark ? HIGH : LOW);
  Serial.print("Ciemno? ");
  Serial.println(isDark ? "TAK" : "NIE");

  // 📈 Przechylenie z akcelerometru
  sensors_event_t event;
  accel.getEvent(&event);
  float tiltAmount = abs(event.acceleration.x) + abs(event.acceleration.y);

if (tiltAmount > TILT_THRESHOLD) {
  if (!isTilting) {
    isTilting = true;
    tiltStartTime = millis();
  } else if (millis() - tiltStartTime > TILT_DURATION && !awaitingAccidentConfirmation) {
    bleChar->setValue("POTENCJALNY_WYPADEK");
    bleChar->notify();
    tone(BUZZER_PIN, 1500, 1000); // Ostrzeżenie
    awaitingAccidentConfirmation = true;
    accidentQueryStart = millis();
  }
} else {
  isTilting = false;
}

if (awaitingAccidentConfirmation) {
  if (bleCommand == "OK" || bleCommand == "NIE WYPADŁEM") {
    awaitingAccidentConfirmation = false;
    bleCommand = "";
    Serial.println("Wypadek odwołany przez użytkownika.");
  } else if (millis() - accidentQueryStart > accidentResponseWindow) {
    bleChar->setValue("WYPADEK_POTWIERDZONY");
    bleChar->notify();
    Serial.println("Brak odpowiedzi. Wypadek potwierdzony.");
    awaitingAccidentConfirmation = false;
  }
}

  // 🚴 Pomiar prędkości
  static unsigned long lastMeasureTime = 0;
  static float speedSamples[HALL_SAMPLE_NUM] = {0};
  static int sampleIndex = 0;

  if (millis() - lastMeasureTime > 2000) {
    float speed = (hallPulseCount * WHEEL_CIRC_CM) / 2000.0 * 1000.0 / 100.0;
    speedSamples[sampleIndex] = speed;
    sampleIndex = (sampleIndex + 1) % HALL_SAMPLE_NUM;
    hallPulseCount = 0;
    lastMeasureTime = millis();

    float avg = 0;
    for (int i = 0; i < HALL_SAMPLE_NUM; i++) avg += speedSamples[i];
    avg /= HALL_SAMPLE_NUM;
    String msg = "Prędkość średnia: " + String(avg) + " m/s";
    bleChar->setValue(msg.c_str());
    bleChar->notify();
  }

  if (millis() - lastBlinkTime > blinkInterval) {
  lastBlinkTime = millis();
  blinkState = !blinkState;

if (leftBlinking) {
      setLEDs(LEFT_LED1_PIN, LEFT_LED2_PIN, LEFT_LED3_PIN, blinkState);
    }

    if (rightBlinking) {
      setLEDs(RIGHT_LED1_PIN, RIGHT_LED2_PIN, RIGHT_LED3_PIN, blinkState);
    }

  delay(100);
  }
}

