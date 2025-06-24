//Arduino #2: координаты по I2C
 #include <SoftwareWire.h>
 #include <SoftwareSerial.h>
 #include <TinyGPS++.h>
 #define I2C_SLAVE_ADDR 8
 SoftwareWire i2cWire(19, 18); // SDA=19, SCL=18
 SoftwareSerial gpsSerial(4, 3); // RX=4, TX=3
 TinyGPSPlus gps;
float rawLat = 0.0, rawLng = 0.0;
void onI2CRequest() {
uint8_t buf[8];
memcpy(buf, &rawLat, 4);
memcpy(buf+4, &rawLng, 4);
 i2cWire.write(buf, 8);
}
void setup() {
 Serial.begin(9600);
 gpsSerial.begin(9600);
 i2cWire.begin(I2C_SLAVE_ADDR);
 i2cWire.onRequest(onI2CRequest);
}
void loop() {
 while (gpsSerial.available()) {
 char c = gpsSerial.read();
 if (gps.encode(c) && gps.location.isValid()) {
 rawLat = gps.location.lat();
 rawLng = gps.location.lng();
 Serial.print("Raw GPS: ");
 Serial.print(rawLat, 6);
 Serial.print(", ");
 Serial.println(rawLng, 6);
 }
}
}