
/*
#include <Arduino.h>
#include <BMP280.h>
#include "HCSR04.h"

BMP280 bmp280;

unsigned long previousMillis = 0; // Zmienna do przechowywania poprzedniego czasu
const long interval = 1000;       // Interwał czasowy w milisekundach (tutaj 1000 ms = 1 sekunda)

byte triggerPin = 11;
byte echoPin = 12;

void setup() {
  Serial.begin(9600);
  HCSR04.begin(triggerPin, echoPin);
  delay(10);
  Serial.println("BMP280 example");
  Wire.begin(); //Dołącz do magistrali I2C
  bmp280.begin();
  
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




  Serial.print("/");
  Serial.print("1: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  Serial.println("---");
  delay(250);
}
*/
#define SAMPLE_SIZE 10 // Liczba próbek do uśrednienia
#define STORM_THRESHOLD 5 // Wartość progowa dla wykrycia sztormu (do dostosowania)
#define SAMPLING_INTERVAL 1000 // Interwał pomiędzy próbkowaniem w milisekundach (do dostosowania)




#include <Arduino.h>
#include <BMP280.h>
#include "HCSR04.h"




BMP280 bmp280;
#define DHTTYPE    DHT11     // DHT 11
unsigned long previousMillis = 0; // Zmienna do przechowywania poprzedniego czasu
const long interval = 1000;       // Interwał czasowy w milisekundach (tutaj 1000 ms = 1 sekunda)

byte triggerPin = 11;
byte echoPin = 12;



int Temperature_in = 0;
int Temperature_out = 0;
int Pressure = 0;
int Humidity = 0;
uint16_t Distance = 0;
boolean Status = 0;

 int t1 = 0;
int h1 = 0;

unsigned long lastSampleTime = 0; // Czas ostatniego pobrania próbki

bool detectStorm(float* temperature, float* pressure, float* humidity) {
    float tempSum = 0, pressureSum = 0, humiditySum = 0;

    // Pobieranie próbek
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        tempSum += temperature[i];
        pressureSum += pressure[i];
        humiditySum += humidity[i];
    }

    // Obliczanie średnich wartości
    float avgTemp = tempSum / SAMPLE_SIZE;
    float avgPressure = pressureSum / SAMPLE_SIZE;
    float avgHumidity = humiditySum / SAMPLE_SIZE;

    // Analiza danych
    if ((avgPressure < STORM_THRESHOLD) && (avgTemp < STORM_THRESHOLD) && (avgHumidity > STORM_THRESHOLD)) {
        // Znaleziono potencjalne oznaki sztormu
        return true;
    } else {
        // Brak oznak sztormu
        return false;
    }
}



void serial_data(uint16_t temperature_in, uint16_t temperature_out, uint16_t pressure, uint8_t humidity, uint16_t distance, uint8_t status)
  {
    /*
    uint8_t buffer[16];

    buffer[0] = 0xAA;
    buffer[1] = 0xAA;
    buffer[2] = 0xAA;
    buffer[3] = 0xA5;

    buffer[4] = temperature_in & 0xFF00 >> 8;
    buffer[5] = temperature_in & 0xFF;

    buffer[6] = temperature_out & 0xFF00 >> 8;
    buffer[7] = temperature_out & 0xFF;

    buffer[8] = pressure & 0xFF00 >> 8;
    buffer[9] = pressure & 0xFF;

    buffer[10] = humidity;

    buffer[11] = distance & 0xFF00 >> 8;
    buffer[12] = distance & 0xFF;

    buffer[13] = status;

    uint16_t crc = temperature_in + temperature_out + pressure + humidity + distance + status;

    buffer[14] = crc & 0xFF00 >> 8;
    buffer[15] = crc & 0xFF;

    Serial.print((char*)buffer);
    */

    if(temperature_in > -1)
    {
    Serial.print(" ");
    Serial.print(temperature_in);
    Serial.print("\xC2\xB0");
    Serial.print("/");
    }
    else
    {
    Serial.print(temperature_in);
    Serial.print("\xC2\xB0");
    Serial.print("/");
    }

    if(temperature_out > -1)
    {
    Serial.print(temperature_out);
    Serial.print(" \xC2\xB0");
    Serial.print("/");
    }
    else
    {
    Serial.print(" ");
    Serial.print(temperature_out);
    Serial.print("\xC2\xB0");
    Serial.print("/");
    }
    Serial.print(pressure);
    Serial.print("hPa");
    Serial.print("/");

    Serial.print(humidity);
    Serial.print("%RH");
    Serial.print("/");

    Serial.print(distance);
    Serial.print("cm");
    Serial.print("/");

    Serial.print(status);
    Serial.print("/");
  }






void setup() {
  Serial.begin(9600);
  HCSR04.begin(triggerPin, echoPin);
  delay(10);
  Serial.println("BMP280 example");
  Wire.begin(); //Dołącz do magistrali I2C
  bmp280.begin();


}

void loop() {




  unsigned long currentMillis = millis(); // pobieranie aktualnego czasu
  double* distances = HCSR04.measureDistanceCm();
  // Sprawdź, czy minęło wystarczająco dużo czasu od ostatniego odczytu
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Zapisz aktualny czas jako poprzedni

    // pobieranie wartości ciśnienia i temperatury
    uint32_t pressure = bmp280.getPressure();
    float temperature = bmp280.getTemperature();

    

    // Attempt to read the temperature and humidity values from the DHT11 sensor.
               
                  

   
    Temperature_out = int(temperature)+7;
    Pressure = int(pressure);
    
    
    Status = 0;



    Temperature_out += 273;
    Temperature_in += 273;

    uint16_t t10 = 

    Distance = static_cast<uint16_t>(*distances);
    serial_data(Temperature_out, Temperature_out, Pressure, 65, Distance, Status); //wyświetlanie danych w serial monitorze
  


  }
  

  




  unsigned long currentMillis2 = millis(); // Aktualny czas

    // Sprawdzanie, czy upłynął odpowiedni interwał czasowy od ostatniego pobrania próbek
    if (currentMillis2 - lastSampleTime >= SAMPLING_INTERVAL) {
        lastSampleTime = currentMillis2; // Aktualizacja czasu ostatniego pobrania próbek

        // Pobieranie danych z czujników (temperatura, ciśnienie, wilgotność)
        float temperature[SAMPLE_SIZE];
        float pressure[SAMPLE_SIZE];
        float humidity[SAMPLE_SIZE];

        // Wypełnianie tablic danych
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            // Pobieranie danych z czujników 
             temperature[i] = Temperature_out;
             pressure[i] = Pressure;
             humidity[i] = Humidity;
        }

        // Wywołanie funkcji do detekcji sztormu
        bool isStorm = detectStorm(temperature, pressure, humidity);

        // Obsługa rezultatu analizy
        if (isStorm) {
            Status = 1;
        } else {
            Status= 0;
        }
    }

}









