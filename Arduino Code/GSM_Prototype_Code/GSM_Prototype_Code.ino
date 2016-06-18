#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "Adafruit_FONA.h"
#include <math.h>
#include "DHT.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN            7

#define NUMPIXELS      12

#define DHTPIN 8
#define FONA_RX 0
#define FONA_TX 1
#define FONA_RST 4

#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE,27);
// this is a large buffer for replies
char replybuffer[255];
char *s1 = "00:40-";
char *s2 = "15:40-";
char *s3 = "30:40-";
char *s4 = "45:40-";
int sender = 0;

//this is the communication for the fona
#define HWSERIAL Serial1
HardwareSerial fonaSerial = Serial1;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

//check version of arduino 
#if (ARDUINO >= 100)
 #include "Arduino.h"
  #ifdef SERIAL_PORT_USBVIRTUAL
    #include <HardwareSerial.h>
  #else
    #include <SoftwareSerial.h>
  #endif
#else
 #include "WProgram.h"
 #include <NewSoftSerial.h>
#endif

const int xpin = A4;                  // x-axis of the accelerometer
const int ypin = A5;                  // y-axis of the accelerometer
const int zpin = A6;                  // z-axis of the accelerometer
char buffer[23];

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

//check current battery
uint16_t currentVoltage = 0;

//neopixel object setup
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500;

//debounced push button code
const int buttonPin = 16;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int zedr = 0;
int state = 0;
int previousState = -1;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
int zeder = 5;
int change = 1;
int maxVal = 50;
int zedtestingErr = 1;

//Store the humidity, temperature (Celcius and Fahrenheit), and heatindex
float h;
float t;
float f;
float hi;

void setup() {
  //debugging (Uncomment for production)
  while (!Serial);
  
  Serial.begin(9600);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial.begin(4800);
  if (! fona.begin(fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default: 
      Serial.println(F("???"));
      break;
  }
  
  
  // Print module IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
  
  pixels.begin();
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  dht.begin();
  initalize();
}

void repeatMe() {
   zedtestingErr = 1;
}

//get if the GPS and Cellular are working
int networkStatus(void){
  uint8_t n = fona.getNetworkStatus();
  
  while(n != 1){
      n = fona.getNetworkStatus();
      Serial.print(F("Network status "));
      Serial.print(n);
      Serial.print(F(": "));
      if (n == 0) Serial.println(F("Not registered"));
      if (n == 1) Serial.println(F("Registered (home)"));
      if (n == 2) Serial.println(F("Not registered (searching)"));
      if (n == 3) Serial.println(F("Denied"));
      if (n == 4) Serial.println(F("Unknown"));
      if (n == 5) Serial.println(F("Registered roaming"));
      delay(2000);
  }
  zeder = 0;
      while(zeder <= maxVal)
      {
        for(int i=0;i<NUMPIXELS;i++){
          pixels.setPixelColor(i, pixels.Color(0,round(0.8*zeder),round(0.2*zeder))); // Moderately bright green color.
          delay(5); 
        }
        zeder++;
        pixels.show();
      }
      while(zeder >= 0)
      {
         for(int i=0;i<NUMPIXELS;i++){
            pixels.setPixelColor(i, pixels.Color(0,round(0.8*zeder),round(0.2*zeder))); // Moderately bright green color.
            delay(5);
         }
         zeder--;
         pixels.show();
      }
  return 1;
}

//turn on the GPS chip
int turnOnGPS(void){
  // turn GPS on
    if (!fona.enableGPS(true)){
      Serial.println(F("Failed to turn on"));
    }
    if (!fona.enableNetworkTimeSync(true)){
          Serial.println(F("Failed to enable"));
    }
    delay(3000);
      
      int8_t stat = 1;
      // check GPS fix
      
      while((stat !=  2) && (stat != 3))
      {
        stat = fona.GPSstatus();
        if (stat < 0)
          Serial.println(F("Failed to query"));
        if (stat == 0) Serial.println(F("GPS off"));
        if (stat == 1) Serial.println(F("No fix"));
        if (stat == 2) Serial.println(F("2D fix"));
        if (stat == 3) Serial.println(F("3D fix"));
        
        zeder = 2;
      while(zeder <= maxVal)
      {
        for(int i=0;i<NUMPIXELS;i++){
          pixels.setPixelColor(i, pixels.Color(round(0.3*zeder),round(0.3*zeder),round(0.3*zeder))); // Moderately bright green color.
          delay(1); 
        }
        zeder++;
        pixels.show();
      }
      while(zeder >= 2)
      {
         for(int i=0;i<NUMPIXELS;i++){
            pixels.setPixelColor(i, pixels.Color(round(0.3*zeder),round(0.3*zeder),round(0.3*zeder))); // Moderately bright green color.
            delay(1);
         }
         zeder--;
         pixels.show();
      }
        
        delay(2000);
      }
      
      zeder = 0;
      while(zeder <= maxVal)
      {
        for(int i=0;i<NUMPIXELS;i++){
          pixels.setPixelColor(i, pixels.Color(0,round(0.1*zeder),round(0.9*zeder))); // Moderately bright green color.
          delay(5); 
        }
        zeder++;
        pixels.show();
      }
      while(zeder >= 0)
      {
         for(int i=0;i<NUMPIXELS;i++){
            pixels.setPixelColor(i, pixels.Color(0,round(0.1*zeder),round(0.9*zeder))); // Moderately bright green color.
            delay(5);
         }
         zeder--;
         pixels.show();
      }
      return stat;
}

//turn on the volume
void setHeadSet(void){
        // set to headphone jack
        if (! fona.setAudio(FONA_HEADSETAUDIO)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        fona.setMicVolume(FONA_HEADSETAUDIO, 15);
        
         // set volume
        flushSerial();
        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          Serial.print(F("Set Vol [0-8] "));
        } else {
          Serial.print(F("Set Vol % [0-100] "));
        }
        uint8_t vol = 50;
        //Serial.println();
        if (! fona.setVolume(vol)) {
          Serial.println(F("Failed"));
        } else {
          //Serial.println(F("OK!"));
        }
        
        //play a tone
        flushSerial();
        Serial.print(F("Play tone #"));
        uint8_t kittone = 20;
        Serial.println();
        // play for 1 second (1000 ms)
        if (! fona.playToolkitTone(kittone, 1000)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        
        zeder = 0;
      while(zeder <= maxVal)
      {
        for(int i=0;i<NUMPIXELS;i++){
          pixels.setPixelColor(i, pixels.Color(round(0.7*zeder),round(0.3*zeder),0)); // Moderately bright green color.
          delay(5);
        }
        zeder++;
        pixels.show();
      }
      while(zeder >= 0)
      {
         for(int i=0;i<NUMPIXELS;i++){
            pixels.setPixelColor(i, pixels.Color(round(0.7*zeder),round(0.3*zeder),0)); // Moderately bright green color.
            delay(5);
         }
         zeder--;
         pixels.show();
      }
}

//initializing script
void initalize(void) {
  //light startup
  for(int i=0;i<NUMPIXELS;i++){
    if(state == 0){
      pixels.setPixelColor(i, pixels.Color(0,zeder,0)); // Moderately bright green color.
    } else if(state == 1) {
      pixels.setPixelColor(i, pixels.Color(zeder,0,0));
    } else if(state == 2){
      pixels.setPixelColor(i, pixels.Color(0,0,zeder));
    }
    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).
    
  }
  zeder = maxVal;
  zedr = 1;
  //newtork status up
  networkStatus();
  //set mic volume
  setHeadSet();
  //turn on the GPS
  turnOnGPS();
}


//get battery reading
int getBatteryVoltage(void){
  uint16_t vbat;
  if (! fona.getBattPercent(&vbat)) {
     Serial.println(F("Failed to read Batt"));
     return -1;
  } else {
     Serial.print(F("VPct = "));
     Serial.print(vbat);
     Serial.println(F("%"));
     return (vbat+0);
  }
}

void loop() {
  delay(2000);
   //put your main code here, to run repeatedly:
   //get voltage
  uint16_t adc;
  getBatteryVoltage();
  ReadTempHumiditySensor();
  readAccloremeter();
  
  Serial.println();
  Serial.println();
  char gpsdata[120];
  fona.getGPS(0, gpsdata, 120);
  if (type == FONA808_V1)
      Serial.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
      Serial.println(gpsdata);
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
   
   fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
   
   if((strstr(buffer, s1) != NULL) || (strstr(buffer, s2) != NULL) || (strstr(buffer, s3) != NULL) || (strstr(buffer, s4) != NULL) || (zedtestingErr == 1)){
        char sendto[21];
        char message[141];
        flushSerial();
        //Serial.print(F("Send to #"));
        sendto[0] = '5';
        sendto[1] = '1';
        sendto[2] = '2';
        sendto[3] = '9';
        sendto[4] = '8';
        sendto[5] = '3';
        sendto[6] = '1';
        sendto[7] = '7';
        sendto[8] = '6';
        sendto[9] = '7';
        Serial.println(sendto);
        Serial.print(F("Type out one-line message (140 char): "));
        strncpy(message, gpsdata, 120);
        
        Serial.println(message);
        if (!fona.sendSMS(sendto, message)) {
           for(int i=0;i<NUMPIXELS;i++){
              pixels.setPixelColor(i, pixels.Color(20,0,0));
           }
           pixels.show();
          delay(100);
           for(int i=0;i<NUMPIXELS;i++){
              pixels.setPixelColor(i, pixels.Color(0,0,0));
           }
          pixels.show();
        } else {
           for(int i=0;i<NUMPIXELS;i++){
              pixels.setPixelColor(i, pixels.Color(0,20,20));
           }
          pixels.show();
          delay(100);
           for(int i=0;i<NUMPIXELS;i++){
              pixels.setPixelColor(i, pixels.Color(0,0,0));
           }
          pixels.show();
        }
    zedtestingErr = 2;
  }
  //network status
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  } 
  
  if((buttonState != lastButtonState) && (millis() - lastDebounceTime) > debounceDelay)
  {
     state = (state+1)%3; 
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    buttonState = reading;
    
    
  }
  if(state != previousState){
    change++;
  }
  
  
  if(state != previousState && change == 2){
    change = 0;
    while(zeder <= maxVal)
    {
      for(int i=0;i<NUMPIXELS;i++){
     
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
         
        if(state == 0){
          pixels.setPixelColor(i, pixels.Color(0,zeder,0)); // Moderately bright green color.
        } else if(state == 1) {
          pixels.setPixelColor(i, pixels.Color(zeder,0,0));
        } else if(state == 2){
          pixels.setPixelColor(i, pixels.Color(0,0,zeder));
        }
        pixels.show(); // This sends the updated pixel color to the hardware.
    
        //delay(delayval); // Delay for a period of time (in milliseconds).
    
      }
      zeder++;
      delay(10);
    }
    while(zeder >= 0)
    {
      for(int i=0;i<NUMPIXELS;i++){
     
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
         
        if(state == 0){
          pixels.setPixelColor(i, pixels.Color(0,zeder,0)); // Moderately bright green color.
        } else if(state == 1) {
          pixels.setPixelColor(i, pixels.Color(zeder,0,0));
        } else if(state == 2){
          pixels.setPixelColor(i, pixels.Color(0,0,zeder));
        }
        pixels.show(); // This sends the updated pixel color to the hardware.
    
        //delay(delayval); // Delay for a period of time (in milliseconds).
    
      }
      zeder--;
      delay(10);
    }
  }
  zeder = 0;
  if(zedr == 1)
  {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    zedr++;
  }
  
  previousState = state;
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
   
}

int readAccloremeter(){
  Serial.println();
  Serial.println();
  Serial.println("Accloremeter");
  // print the sensor values:
  Serial.print(analogRead(xpin));
  // print a tab between values:
   Serial.print("\t");
   Serial.print(analogRead(ypin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print(analogRead(ypin));
  Serial.println();
  // delay before next reading:
  delay(100);
}

int ReadTempHumiditySensor(){
  // Reading temperature or humidity takes about 250 milliseconds
  // Sensor readings may also be up to 2 seconds, its a very slow sensor
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return 1;
  }
  
  hi = dht.computeHeatIndex(f, h);
  
  Serial.print("Humidity: "); 
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: "); 
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hi);
  Serial.println(" *F");
  return 0;
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  return 'a';
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //loop
  }
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      break;
    }

    if (timeoutvalid && timeout == 0) {
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;
  return buffidx;
}
