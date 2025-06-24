// 1. Основная ардуино
#include <Wire.h>
#include <SoftwareWire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <math.h>

// I2C-адрес GPS-станции на роботе
#define GPS_SLAVE_ADDR 0x08

// Адреса моторных драйверов
#define vbok    0x60
#define vpered  0x61

// Пины
#define SNEK_PIN    3
#define BUTTON_GPS  9
#define BUTTON_BT   10
#define BATTERY_PIN A0

// I2C и Serial
SoftwareWire    myWire(19, 18);
SoftwareSerial  mySerial(4, 3);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// nRF24L01
RF24 radio(8, 7);
const byte address[6] = "00001";

// Режимы
enum Mode { BLUETOOTH, GPS_AUTO };
Mode currentMode = BLUETOOTH;

// GPS-данные
struct GPSData { float lat, lng; };
GPSData raw{}, base{}, current{};
bool gpsAvailableRaw = false, baseAvailable = false;

// Периметр
const int MAX_PERIM = 200;
GPSData perim[MAX_PERIM];
int perimCount = 0;
bool perimMapped = false;
GPSData startPoint;
unsigned long lastRecord = 0;
const unsigned long RECORD_INTERVAL = 2000;
const float RECORD_DIST = 1.0;

// Змейка
const int MAX_SNAKE = 500;
GPSData snake[MAX_SNAKE];
int snakeCount = 0, snakeIdx = 0;
bool snakeStarted = false, returnHome = false;

// Батарея
unsigned long lastBatteryUpdate = 0;
const int BATTERY_UPDATE_INTERVAL = 1000;
float batteryPercent = 100.0;
const float BATTERY_THRESHOLD = 15.0;

// Кнопки (пред. состояние)
bool prevBtPressed = false, prevGpsPressed = false;

// Прототипы
float distanceBetween(float, float, float, float);
float bearingBetween(float, float, float, float);
void set_output_level(int, bool, int);
void goForward(), turnLeft(), stopAll();
void driveHeading(float);
void goTo(const GPSData &);
void updateBatteryStatus();
void generateSnake();

void setup() {
  pinMode(BUTTON_BT, INPUT_PULLUP);
  pinMode(BUTTON_GPS, INPUT_PULLUP);
  pinMode(SNEK_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  myWire.begin();
  Wire.begin();
  lcd.begin(16,2);
  lcd.backlight();
  radio.begin();
  radio.openReadingPipe(1,address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  lastRecord = millis();
  current = raw = base = {0,0};
}

void loop() {
  // 1) GPS по I2C
  gpsAvailableRaw = false;
  if (myWire.requestFrom(GPS_SLAVE_ADDR,8) == 8) {
    myWire.readBytes((char*)&raw.lat,4);
    myWire.readBytes((char*)&raw.lng,4);
    gpsAvailableRaw = true;
  }
  // 2) Поправки по nRF
  baseAvailable = false;
  if (radio.available()) {
    radio.read(&base,sizeof(base));
    baseAvailable = true;
  }
  // 3) Усреднение
  if (gpsAvailableRaw) {
    if (baseAvailable) {
      current.lat = (raw.lat + base.lat)*0.5;
      current.lng = (raw.lng + base.lng)*0.5;
    } else {
      current = raw;
    }
  }
  // 4) Батарея
  updateBatteryStatus();
  if (batteryPercent < BATTERY_THRESHOLD && currentMode==GPS_AUTO && !returnHome) {
    snakeStarted = false;
    returnHome   = true;
    digitalWrite(SNEK_PIN, LOW);
    lcd.clear(); lcd.print("Low Batt! Return");
    delay(200);
  }
  // 5) Кнопки
  bool btPressed  = digitalRead(BUTTON_BT)==LOW;
  bool gpsPressed = digitalRead(BUTTON_GPS)==LOW;
  if (btPressed && !prevBtPressed) {
    currentMode = BLUETOOTH;
    perimCount = 0; perimMapped = false;
    snakeStarted = returnHome = false;
    lcd.clear(); lcd.print("Mapping Start");
    delay(200);
  }
  if (gpsPressed && !prevGpsPressed) {
    if (currentMode==BLUETOOTH && perimMapped) {
      currentMode=GPS_AUTO; snakeStarted=true; snakeIdx=0;
      generateSnake();
      lcd.clear(); lcd.print("Snake Start"); delay(200);
    }
    else if (currentMode==GPS_AUTO && snakeStarted && !returnHome) {
      snakeStarted=false; returnHome=true;
      digitalWrite(SNEK_PIN, LOW);
      lcd.clear(); lcd.print("Return Home"); delay(200);
    }
  }
  prevBtPressed=btPressed; prevGpsPressed=gpsPressed;
  // 6) Логика
  if (currentMode==BLUETOOTH) {
    lcd.setCursor(0,0); lcd.print("BT Mapping ");
    if (mySerial.available()) {
      char c = mySerial.read();
      switch(c){
        case 'F': goForward(); break;
        case 'R': set_output_level(1200,true,vpered);
                  set_output_level(400,false,vbok); break;
        case 'L': set_output_level(2000,false,vbok);
                  set_output_level(1200,false,vpered); break;
        case 'S': stopAll(); break;
        case 'C': digitalWrite(SNEK_PIN,HIGH); break;
        case 'D': digitalWrite(SNEK_PIN,LOW); break;
      }
    }
    if (gpsAvailableRaw && !perimMapped) {
      if (perimCount==0) {
        startPoint=current; perim[perimCount++]=current; lastRecord=millis();
      }
      else if (millis()-lastRecord>=RECORD_INTERVAL &&
               distanceBetween(current.lat,current.lng,
                               perim[perimCount-1].lat,
                               perim[perimCount-1].lng)>=RECORD_DIST) {
        if (perimCount<MAX_PERIM) {
          perim[perimCount++]=current; lastRecord=millis();
        }
      }
      if (perimCount>=10 &&
          distanceBetween(current.lat,current.lng,
                          startPoint.lat,startPoint.lng)<RECORD_DIST) {
        perimMapped=true; stopAll();
        lcd.setCursor(0,1); lcd.print("Map Complete");
      }
    }
  }
  else { // GPS_AUTO
    if (!perimMapped) {
      lcd.clear(); lcd.print("No Perimeter"); stopAll();
    }
    else if (snakeStarted) {
      lcd.setCursor(0,0); lcd.print("Snake Mode ");
      if (snakeIdx<snakeCount) {
        goTo(snake[snakeIdx]);
        if (distanceBetween(current.lat,current.lng,
                            snake[snakeIdx].lat,
                            snake[snakeIdx].lng)<RECORD_DIST) {
          snakeIdx++; delay(200);
        }
      } else {
        stopAll(); lcd.setCursor(0,1); lcd.print("Snake Done");
      }
    }
    else if (returnHome) {
      lcd.setCursor(0,0); lcd.print("Returning ");
      goTo(startPoint);
      if (distanceBetween(current.lat,current.lng,
                          startPoint.lat,startPoint.lng)<RECORD_DIST) {
        stopAll(); lcd.setCursor(0,1); lcd.print("At Start   ");
      }
    }
    else {
      lcd.setCursor(0,0); lcd.print("Ready GPS  "); stopAll();
    }
  }
  delay(50);
}


float distanceBetween(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2-lat1), dLon = radians(lon2-lon1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2)) *
            sin(dLon/2)*sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

float bearingBetween(float lat1, float lon1, float lat2, float lon2) {
  float dLon=radians(lon2-lon1);
  lat1=radians(lat1); lat2=radians(lat2);
  float y = sin(dLon)*cos(lat2);
  float x = cos(lat1)*sin(lat2) -
            sin(lat1)*cos(lat2)*cos(dLon);
  return fmod(degrees(atan2(y,x))+360.0,360.0);
}

void set_output_level(int level, bool save, int addr) {
  uint8_t cmd[3]={ save?0x60:0x40, uint8_t(level>>4), uint8_t((level&0x0F)<<4) };
  Wire.beginTransmission(addr);
    Wire.write(cmd,3);
  Wire.endTransmission();
  delayMicroseconds(100);
}

void goForward(){ set_output_level(1200,false,vbok); set_output_level(1400,false,vpered); }
void turnLeft(){ set_output_level(2000,false,vbok); set_output_level(1200,false,vpered); }
void stopAll(){ set_output_level(1220,true,vbok); set_output_level(1070,false,vpered); }
void driveHeading(float b){ goForward(); }

void goTo(const GPSData &wp) {
  float d=distanceBetween(current.lat,current.lng,wp.lat,wp.lng);
  if (d<RECORD_DIST) stopAll();
  else {
    float brg=bearingBetween(current.lat,current.lng,wp.lat,wp.lng);
    driveHeading(brg);
  }
}

void updateBatteryStatus() {
  if (millis()-lastBatteryUpdate< BATTERY_UPDATE_INTERVAL) return;
  lastBatteryUpdate=millis();
  int v=analogRead(BATTERY_PIN);
  float voltage=v*(5.0/1023.0)*(91.0+10.0)/10.0;
  batteryPercent=map(constrain(voltage,34,42),30,42,0,100);
  lcd.setCursor(10,0);
  lcd.print(int(batteryPercent)); lcd.print("% ");
}

void generateSnake() {
  float latMin=perim[0].lat, latMax=perim[0].lat;
  float lngMin=perim[0].lng, lngMax=perim[0].lng;
  for(int i=1;i<perimCount;i++){
    latMin=min(latMin,perim[i].lat);
    latMax=max(latMax,perim[i].lat);
    lngMin=min(lngMin,perim[i].lng);
    lngMax=max(lngMax,perim[i].lng);
  }
  float step=RECORD_DIST/111000.0;
  bool dir=true;
  snakeCount=0;
  for(float la=latMin; la<=latMax; la+=step){
    if(dir){
      snake[snakeCount++]={la,lngMin};
      snake[snakeCount++]={la,lngMax};
    } else {
      snake[snakeCount++]={la,lngMax};
      snake[snakeCount++]={la,lngMin};
    }
    dir=!dir;
    if(snakeCount>=MAX_SNAKE-2) break;
  }
}