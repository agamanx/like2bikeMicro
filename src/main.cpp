#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h> // Wymaga biblioteki Adafruit_Sensor i Adafruit_ADXL345
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Piny
#define BUZZER_PIN      26
#define LDR_PIN         34
#define HALL_SENSOR_PIN 35
#define RELAY_PIN       25
#define LED_LEFT     32
#define LED_RIGHT     33

// 🌡️ Progi i ustawienia
#define LDR_THRESHOLD   1500
#define TILT_THRESHOLD  20     // Nachylenie (X lub Y) w m/s^2
#define TILT_DURATION   5000   // czas utrzymania (ms)
#define HALL_SAMPLE_NUM 5
#define WHEEL_CIRC_CM   210.0  // np. koło 66mm średnicy (~210mm obwodu)

// 🧭 BLE
BLECharacteristic *bleChar;
bool bleClientConnected = false;
String bleCommand = "";

// 📈 Zmienna przechylania
unsigned long tiltStartTime = 0;
bool isTilting = false;

// 🧲 Czujnik Halla
volatile int hallPulseCount = 0;
unsigned long hallLastTime = 0;

// 📦 Akcelerometr
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// 📡 BLE Callback
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxVal = pCharacteristic->getValue();
    bleCommand = String(rxVal.c_str());
    Serial.print("BLE Received: ");
    Serial.println(bleCommand);
  }
};

// 💡 Przerwanie Halla
void IRAM_ATTR hallInterrupt() {
  hallPulseCount++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Ustawienia pinów
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallInterrupt, RISING);

  if (!accel.begin()) {
    Serial.println("Brak ADXL345!");
    while (1);
  }

  // 🔧 BLE Init
  BLEDevice::init("ESP32_Module");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789abc");
  bleChar = pService->createCharacteristic("abcd", BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  bleChar->addDescriptor(new BLE2902());
  bleChar->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("Uruchomione.");
}

// 🔁 Pętla główna
void loop() {
  // 🌘 LDR – włącz Power LED przez przekaźnik
  int ldrVal = analogRead(LDR_PIN);
  digitalWrite(RELAY_PIN, ldrVal < LDR_THRESHOLD ? HIGH : LOW);

  // 📈 Akcelerometr – przechylenie
  sensors_event_t event;
  accel.getEvent(&event);
  float tiltAmount = abs(event.acceleration.x) + abs(event.acceleration.y);

  if (tiltAmount > TILT_THRESHOLD) {
    if (!isTilting) {
      isTilting = true;
      tiltStartTime = millis();
    } else if (millis() - tiltStartTime > TILT_DURATION) {
      tone(BUZZER_PIN, 1500, 500);
      bleChar->setValue("Tilt detected!");
      bleChar->notify();
      isTilting = false;
    }
  } else {
    isTilting = false;
  }

  // ⏱️ Pomiar prędkości przez Hall Sensor
  static unsigned long lastMeasureTime = 0;
  static float speedSamples[HALL_SAMPLE_NUM] = {0};
  static int sampleIndex = 0;

  if (millis() - lastMeasureTime > 2000) {
    float speed = (hallPulseCount * WHEEL_CIRC_CM) / 2000.0 * 1000.0 / 100.0; // cm/s → m/s
    speedSamples[sampleIndex] = speed;
    sampleIndex = (sampleIndex + 1) % HALL_SAMPLE_NUM;
    hallPulseCount = 0;
    lastMeasureTime = millis();

    // Liczymy średnią i wysyłamy przez BLE
    float avg = 0;
    for (int i = 0; i < HALL_SAMPLE_NUM; i++) avg += speedSamples[i];
    avg /= HALL_SAMPLE_NUM;
    String msg = "Prędkość średnia: " + String(avg) + " m/s";
    bleChar->setValue(msg.c_str());
    bleChar->notify();
  }

  // ↔️ Kierunkowskazy przez BLE
  if (bleCommand == "left") {
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    bleCommand = "";
  } else if (bleCommand == "right") {
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, HIGH);
    bleCommand = "";
  } else if (bleCommand == "off") {
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    bleCommand = "";
  }

  delay(100);
}
