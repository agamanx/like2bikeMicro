#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h> // Wymaga biblioteki Adafruit_Sensor i Adafruit_ADXL345

// Piny
#define BUZZER_PIN      26
#define LDR_PIN         34
#define HALL_SENSOR_PIN 35
#define RELAY_PIN       25
#define LED_STRIP_1     32
#define LED_STRIP_2     33

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Ustawienia pinów
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_STRIP_1, OUTPUT);
  pinMode(LED_STRIP_2, OUTPUT);

  // Wstępny test
  digitalWrite(RELAY_PIN, HIGH);  // Włączenie Power LED przez przekaźnik
  digitalWrite(LED_STRIP_1, HIGH);
  digitalWrite(LED_STRIP_2, HIGH);
  tone(BUZZER_PIN, 1000, 200); // Krótki sygnał dźwiękowy

  delay(500);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_STRIP_1, LOW);
  digitalWrite(LED_STRIP_2, LOW);

  // Inicjalizacja akcelerometru
  if (!accel.begin()) {
    Serial.println("Nie znaleziono ADXL345. Sprawdź połączenia!");
    while (1);
  }
  Serial.println("ADXL345 wykryty!");

  // Czujniki analogowe nie wymagają inicjalizacji
}

void loop() {
  // put your main code here, to run repeatedly:
  // Odczyt LDR
  int ldrValue = analogRead(LDR_PIN);
  Serial.print("LDR: ");
  Serial.println(ldrValue);

  // Odczyt Halla
  int hallValue = analogRead(HALL_SENSOR_PIN);
  Serial.print("Czujnik Halla: ");
  Serial.println(hallValue);

  // Odczyt akcelerometru
  sensors_event_t event;
  accel.getEvent(&event);
  Serial.print("X: "); Serial.print(event.acceleration.x);
  Serial.print(" Y: "); Serial.print(event.acceleration.y);
  Serial.print(" Z: "); Serial.println(event.acceleration.z);

  delay(1000); // co sekundę
}
