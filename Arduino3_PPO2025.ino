//Arduino #3: Стационарная станция (дифференциальная коррекция)

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define GPS_RX_PIN 4
#define GPS_TX_PIN 3
#define RF_CE_PIN  8
#define RF_CSN_PIN 7

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus    gps;
RF24           radio(RF_CE_PIN, RF_CSN_PIN);

const byte rfAddress[6] = "00001";

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  radio.begin();
  radio.openWritingPipe(rfAddress);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c) && gps.location.isValid()) {
      struct { float lat, lng; } msg = {
        gps.location.lat(),
        gps.location.lng()
      };
      radio.write(&msg, sizeof(msg));
      Serial.print("Base GPS: ");
      Serial.print(msg.lat, 6);
      Serial.print(", ");
      Serial.println(msg.lng, 6);
      delay(500);
    }
  }
}