#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <dht.h>
#include <PString.h>

//setup neopixel variables
#define DHT22_PIN 10
#define ss Serial3
#define nss Serial1
#define PIN            9
#define NUMPIXELS      12

//set up accelerometer values
const int xpin = A4;
const int ypin = A5;
const int zpin = A6;

//a time variable
int time = 0;

//the temperature and humidity sensor object
dht DHT;

struct
{
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0};

//the gps module pin
TinyGPS gps;
int pin = 4;
unsigned long durationHIGH;
unsigned long durationLOW;
long durationtot;
boolean fixFound = false;

long sleepit;

//getting the gps latitude and longitude values
float hereGPSflat;
float hereGPSflon;

//connecting te satillite chips
IridiumSBD isbd(nss, 3);
static const int ledPin = 13;

long xrror = 1;

//creating Neopixel Object
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 500;

//setting up the Microcontroller
void setup() {
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  
  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);

  nss.begin(19200);

  isbd.attachConsole(Serial);
  isbd.setPowerProfile(1);
  isbd.begin();
  
  Serial.begin(115200);
  pinMode(pin, INPUT);
  ss.begin(9600);
  
  pixels.begin();
  GPSSetup();
}

//set up gps signal and satillite signal
void GPSSetup(){
  for(int j = 0; j < 25; j = j+1){
        for(int i=0;i<NUMPIXELS;i++){
         pixels.setPixelColor(i, pixels.Color(j,j,j)); // Moderately bright white color.
         pixels.show();
        }
       delay(1);
    }
    delay(2000);
    for(int j = 24; j >= 0; j = j-1){
        for(int i=0;i<NUMPIXELS;i++){
         pixels.setPixelColor(i, pixels.Color(j,j,j)); // Moderately bright white color.
         pixels.show();
        }
       delay(1);
    }
    
  while(!fixFound){
    while(digitalRead(pin) == HIGH){
      durationHIGH = millis();
    }
    while(digitalRead(pin) == LOW){
      durationLOW = millis();
    }
    durationtot = durationLOW - durationHIGH;
    durationtot = abs(durationtot);
    durationtot = durationtot/1000;
    Serial.println(durationtot);
    if(durationtot > 10 && time > 2){
      fixFound = true;
    }
    time++;
    delay(200);
  }
  
    for(int j = 0; j <= 25; j = j+1){
        for(int i=0;i<NUMPIXELS;i++){
         pixels.setPixelColor(i, pixels.Color(0,0,j)); // Moderately bright green color.
         pixels.show();
        }
       delay(1);
    }
    delay(2000);
    
    for(int j = 24; j >= 0; j = j-1){
        for(int i=0;i<NUMPIXELS;i++){
         pixels.setPixelColor(i, pixels.Color(0,0,j)); // Moderately bright green color.
         pixels.show();
        }
       delay(1);
    }
    
  sleepit = millis() - 14400000;
  return;
}

void loop() {
  if((millis()-sleepit) > 14400000){
    int signalQuality = -1;
    isbd.begin();
    int err = isbd.getSignalQuality(signalQuality);
    bool newData = false;
    //gps check
    unsigned long chars;
    unsigned short sentences, failed;
  
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (ss.available())
      {
        char c = ss.read();
        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
    }
  
    if (newData)
    {
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON=");
      Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
      hereGPSflat = flat;
      hereGPSflon = flon;
    }
    
    gps.stats(&chars, &sentences, &failed);
    
    Serial.print("DHT22, \t");

    uint32_t start = micros();
    int chk = DHT.read22(DHT22_PIN);
    uint32_t stop = micros();

    stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        stat.ok++;
        Serial.print("OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
        Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
        Serial.print("Time out error,\t");
        break;
    default:
        stat.unknown++;
        Serial.print("Unknown error,\t");
        break;
    }
    // DISPLAY DATA
    Serial.print(DHT.humidity, 1);
    Serial.print(",\t");
    Serial.print(DHT.temperature, 1);
    Serial.print(",\t");
    Serial.print(stop - start);
    Serial.println();

    if (stat.total % 20 == 0)
    {
        Serial.println("\nTOT\tOK\tCRC\tTO\tUNK");
        Serial.print(stat.total);
        Serial.print("\t");
        Serial.print(stat.ok);
        Serial.print("\t");
        Serial.print(stat.crc_error);
        Serial.print("\t");
        Serial.print(stat.time_out);
        Serial.print("\t");
        Serial.print(stat.connect);
        Serial.print("\t");
        Serial.print(stat.ack_l);
        Serial.print("\t");
        Serial.print(stat.ack_h);
        Serial.print("\t");
        Serial.print(stat.unknown);
        Serial.println("\n");
    }
    
    float xval = analogRead(xpin);
    float yval = analogRead(ypin);
    float zval = analogRead(zpin);
    
    char outBuffer[60];
    
    PString str(outBuffer, sizeof(outBuffer));
    
    str.print(hereGPSflat, 10);
    str.print(hereGPSflon, 10);
    str.print(DHT.humidity, 1);
    str.print(DHT.temperature, 1);
    str.print(xval, 3);
    str.print(yval, 3);
    str.print(zval, 3);
    Serial.print("new: ");
    Serial.print(outBuffer);
    
    err = isbd.sendSBDText(outBuffer);
    Serial.println(signalQuality);
    if (err != 0)
    {
      Serial.print("sendSBDText failed: error ");
      Serial.println(err);
    }
    Serial.println("Hey, it worked!");
    Serial.print("Messages left: ");
    for(int j = 0; j <= 25; j = j+1){
        for(int i=0;i<NUMPIXELS;i++){
         pixels.setPixelColor(i, pixels.Color(0,j,0)); // Moderately bright green color.
         pixels.show();
        }
       delay(1);
    }
    delay(2000);
    
    for(int j = 24; j >= 0; j = j-1){
        for(int i=0;i<NUMPIXELS;i++){
         pixels.setPixelColor(i, pixels.Color(0,j,0)); // Moderately bright green color.
         pixels.show();
        }
       delay(1);
    }
    while(xrror<1000){
      xrror++;
      delay(14400);
    }
    xrror = 1;
    Serial.println(isbd.getWaitingMessageCount());
  }
}

bool ISBDCallback()
{
   digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
   return true;
}
