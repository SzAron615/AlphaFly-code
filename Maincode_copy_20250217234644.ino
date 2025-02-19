#include <Arduino.h>
#include "RadiationWatch.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <DHT.h>
#include <CanSatNeXT.h>

// GPS beállítások
#define RXPin 16  
#define TXPin 17  
#define GPSBaud 9600  

#define MQ_PIN 33  // MQ szenzor analóg kimenet a GPIO33

float sensor_voltage;  // Szenzor feszültség
float RS_air;          // Ellenállás levegőben
float R0 = 10.0;       // Alap kalibrációs ellenállás 

HardwareSerial gpsSerial(1); 
TinyGPSPlus gps;
float latitude, longitude;

// Hőmérséklet szenzor beállítás
#define DHTTYPE DHT11  
#define DHTPIN 15
DHT dht(DHTPIN, DHTTYPE);
bool measureEnabled = true;

RadiationWatch radiationWatch (25,26);

void onRadiation()
{
  Serial.println("A wild gamma ray appeared");
  Serial.print(radiationWatch.uSvh());
  Serial.print(" uSv/h +/- ");
  Serial.println(radiationWatch.uSvhError());
}

void onNoise()
{
  Serial.println("Argh, noise, please stop moving");
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 GPS és szenzor inicializálás...");

  // GPS inicializálás
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("GPS indítása...");

  // DHT11 inicializálás
  Serial.println("DHT11 szenzor indítása...");
  dht.begin();

  // CanSat inicializálás
  CanSatInit();
  CanSatInit(72);
  setRadioChannel(1); // 2.412 GHz

radiationWatch.setup();
  // Register the callbacks.
  radiationWatch.registerRadiationCallback(&onRadiation);
  radiationWatch.registerNoiseCallback(&onNoise);
}
void loop() {
  // Soros parancs figyelése (START / STOP)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 

    if (command == "STOP") {
      measureEnabled = false;
      Serial.println("Mérés leállítva.");
    } else if (command == "START") {
      measureEnabled = true;
      Serial.println("Mérés elindítva.");
    }
  }
  // GPS adat olvasás
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);  

    if (gps.encode(c)) {  
      Serial.print("Látható műholdak: ");
      Serial.println(gps.satellites.value());

      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();

        Serial.print("Szélesség: ");
        Serial.println(latitude, 6);
        Serial.print("Hosszúság: ");
        Serial.println(longitude, 6);
      } else {
        Serial.println("Nincs érvényes GPS-adat. Várakozás...");
      }
    }
  }

  

  if (!measureEnabled) {
    delay(500);
    return;
  }
 // MQ-2 szenzor loop kód
 int sensorValue = analogRead(MQ_PIN);  // ADC érték olvasása
  sensor_voltage = sensorValue * (3.3 / 4095.0);  // ESP32 ADC 12 bites (0-4095)
  
  Serial.print("Feszültség: ");
  Serial.print(sensor_voltage);
  Serial.println(" V");

  // Levegőbeli ellenállás kiszámítása
  RS_air = (3.3 - sensor_voltage) / sensor_voltage;

  // Különböző gázok becsült koncentrációja (ppm)
  float CO_ppm = RS_air / R0 * 50;      // CO becslés
  float LPG_ppm = RS_air / R0 * 100;    // LPG becslés
  float CH4_ppm = RS_air / R0 * 75;     // Metán becslés
  float H2_ppm = RS_air / R0 * 80;      // Hidrogén becslés
  float Smoke_ppm = RS_air / R0 * 60;   // Füst becslés

  Serial.print("CO (Szén-monoxid): ");
  Serial.print(CO_ppm);
  Serial.println(" ppm");

  Serial.print("LPG (Földgáz): ");
  Serial.print(LPG_ppm);
  Serial.println(" ppm");

  Serial.print("CH4 (Metán): ");
  Serial.print(CH4_ppm);
  Serial.println(" ppm");

  Serial.print("H2 (Hidrogén): ");
  Serial.print(H2_ppm);
  Serial.println(" ppm");

  Serial.print("Füst: ");
  Serial.print(Smoke_ppm);
  Serial.println(" ppm");

  Serial.println("----------------------------");


  float temperature = dht.readTemperature();  
  float humidity = dht.readHumidity();        

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Hiba! Nem sikerült kiolvasni az adatokat.");
    return;
  }

  float pressure = readPressure(); 
  float batt = analogReadVoltage(BATT); 

  // Kiírás a soros monitorra
  Serial.print(temperature);
  Serial.print(" °C, ");
  Serial.print(humidity);
  Serial.print(" %, ");
  Serial.print(pressure);
  Serial.print(" hPa, ");
  Serial.print(batt);
  Serial.println(" V");

 radiationWatch.loop();

  // Adatküldés
  char msg[64];
  sprintf(msg, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.6f,%5.6f,", temperature,humidity, pressure, batt,CO_ppm,LPG_ppm,CH4_ppm,H2_ppm,Smoke_ppm,latitude, longitude);
  sendData(msg);

  delay(1000);
}
