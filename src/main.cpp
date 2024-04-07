
#include <Arduino.h>
#include <BMP280.h>
#include "HCSR04.h"
#include <Bonezegei_DHT11.h>
BMP280 bmp280;

unsigned long previousMillis = 0; // Zmienna do przechowywania poprzedniego czasu
const long interval = 1000;       // Interwał czasowy w milisekundach (tutaj 1000 ms = 1 sekunda)

byte triggerPin = 11;
byte echoPin = 12;
Bonezegei_DHT11 dht(2);
void setup() {
  Serial.begin(9600);
  HCSR04.begin(triggerPin, echoPin);
  delay(10);
  Serial.println("BMP280 example");
  Wire.begin(); //Dołącz do magistrali I2C
  bmp280.begin();
  dht.begin();
}

void loop() {
  unsigned long currentMillis = millis(); // Pobierz aktualny czas
  double* distances = HCSR04.measureDistanceCm();
  // Sprawdź, czy minęło wystarczająco dużo czasu od ostatniego odczytu
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Zapisz aktualny czas jako poprzedni

    // Pobierz wartości ciśnienia i temperatury
    uint32_t pressure = bmp280.getPressure();
    float temperature = bmp280.getTemperature();

    // Wyświetl wyniki
  


    Serial.print(temperature);
    Serial.print(" \xC2\xB0");
    Serial.print("/");
    Serial.print(pressure);
    Serial.print("%");
    Serial.print("/");
  }

if (dht.getData()) {                         // get All data from DHT11
    float tempDeg = dht.getTemperature();      // return temperature in celsius
    float tempFar = dht.getTemperature(true);  // return temperature in fahrenheit if true celsius of false
    int hum = dht.getHumidity();               // return humidity
    Serial.print("Temperature: ");
    Serial.print(tempDeg);
    Serial.print("°C\tHumidity: ");
        Serial.print(hum);
    Serial.println("%\n");
  }


  Serial.print("1: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  Serial.println("---");
  delay(250);
}










