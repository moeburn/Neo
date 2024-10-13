#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include <ADS1115_WE.h> 
#include <FastLED.h>
#include <Preferences.h>
#include <Wire.h>


#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#define I2C_ADDRESS 0x48

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);


Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

Preferences preferences;

#define LED_PIN 2

#define NUM_LEDS 5
CRGB leds[NUM_LEDS];



const char* ssid = "mikesnet";
const char* password = "springchicken";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -14400;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 3600;   //Replace with your daylight offset (secs)
int hours, mins, secs;
int zebraR, zebraG, zebraB, menuValue;
int sliderValue = 255;

float tempSHT, humSHT, abshum;
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;
float wifi;

bool buttonstart = false;
bool partymode = false;
bool ledon = false;
bool needssaving = false;


  float current_mA = 0;
  float power_mW = 0;

char auth[] = "19oL8t8mImCdoUqYhfhk6DADL7540f8s";

AsyncWebServer server(80);

WidgetTerminal terminal(V10);

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

BLYNK_WRITE(V10) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("reset");
    terminal.println("ledon");
    terminal.println("ledoff");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
  }
  if (String("reset") == param.asStr()) {
    terminal.println("Restarting...");
    terminal.flush();
    ESP.restart();
  }
  if (String("ledon") == param.asStr()) {
    terminal.println("Turning LED on.");
    terminal.flush();
    ledon = true;
  }
  if (String("ledoff") == param.asStr()) {
    terminal.println("Turning LED on.");
    terminal.flush();
    ledon = false; 
  }

}

BLYNK_WRITE(V18)
{
     zebraR = param[0].asInt();
     zebraG = param[1].asInt();
     zebraB = param[2].asInt();

}

BLYNK_WRITE(V11)
{
  if (param.asInt() == 1) {buttonstart = true;}
  if (param.asInt() == 0) {buttonstart = false;}
}

BLYNK_WRITE(V14)
{
  if (param.asInt() == 1) {partymode = true;}
  if (param.asInt() == 0) {partymode = false;}
}



BLYNK_WRITE(V12)
{
   menuValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V13)
{
   sliderValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V11);
}

void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print(asctime(timeinfo));
}

void goToSleep(){
    //esp_deep_sleep_enable_gpio_wakeup(1, ESP_GPIO_WAKEUP_GPIO_LOW);
    //WiFi.disconnect();
    //delay(1);
    esp_sleep_enable_timer_wakeup(180000000); // 50 sec
    esp_deep_sleep_start(); 
    delay(1000);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

float readChannelmV(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}

float tempinC;

uint8_t startHue = 0;
uint8_t deltaHue = 0;

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  WiFi.begin(ssid, password);
}

void setup(void) {
   tempinC = temperatureRead();
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  bmp.takeForcedMeasurement();
  float presread = bmp.readPressure() / 100.0;
  aht.begin();
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  abshum = (6.112 * pow(2.71828, ((17.67 * temp.temperature)/(temp.temperature + 243.5))) * humidity.relative_humidity * 2.1674)/(273.15 + temp.temperature); //calculate absolute humidity

  adc.init();
  adc.setVoltageRange_mV(ADS1115_RANGE_4096);

  volts2 = 2.0 * readChannel(ADS1115_COMP_0_GND);
  volts3 = 2.0 * readChannel(ADS1115_COMP_1_GND);

  current_mA = readChannelmV(ADS1115_COMP_2_3);



  

  //WiFi.disconnect(true);
  //delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(Wifi_disconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED); 
  WiFi.begin(ssid, password);


    
  while ((WiFi.status() != WL_CONNECTED) && (millis() < 15000)) {
      delay(250);
  }
  wifi = WiFi.RSSI();
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  while ((!Blynk.connected()) && (millis() < 15000)){delay(250);}
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  


  Blynk.virtualWrite(V1, temp.temperature);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V2, tempinC);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V3, volts2);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V4, volts3);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V5, wifi);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V26, presread);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V21, humidity.relative_humidity);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V23, current_mA);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V24, abshum);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V24, abshum);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}

  
  
  
  
  if (buttonstart) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    terminal.println("***Neo 1.1 STARTED***");
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
    printLocalTime();
    terminal.println(volts3,3);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP32 Neo.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    terminal.println("HTTP server started");
    terminal.flush();
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  }
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}

  //delay(500);
  if (!buttonstart){
    pinMode(LED_PIN, INPUT);
    goToSleep();

  }




}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}

  every(5000){
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  bmp.takeForcedMeasurement();
  float presread = bmp.readPressure() / 100.0;
  aht.begin();
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  abshum = (6.112 * pow(2.71828, ((17.67 * temp.temperature)/(temp.temperature + 243.5))) * humidity.relative_humidity * 2.1674)/(273.15 + temp.temperature); //calculate absolute humidity

  adc.init();
  adc.setVoltageRange_mV(ADS1115_RANGE_4096);

  volts2 = 2.0 * readChannel(ADS1115_COMP_0_GND);
  volts3 = 2.0 * readChannel(ADS1115_COMP_1_GND);

  current_mA = readChannelmV(ADS1115_COMP_2_3);


  Blynk.virtualWrite(V1, temp.temperature);
  Blynk.virtualWrite(V2, tempinC);
  Blynk.virtualWrite(V3, volts2);
  Blynk.virtualWrite(V4, volts3);
  Blynk.virtualWrite(V5, wifi);
  Blynk.virtualWrite(V26, presread);
  Blynk.virtualWrite(V21, humidity.relative_humidity);
  Blynk.virtualWrite(V23, current_mA);
  Blynk.virtualWrite(V24, abshum);
  }


    every(10){
      if (!partymode){
        for (int i = 0; i <= NUM_LEDS; i++) {
        leds[i] = CRGB(zebraR, zebraG, zebraB);
        }
        FastLED.setBrightness(sliderValue);
        FastLED.show();
      }
      else {

              fill_rainbow(leds, NUM_LEDS, startHue, deltaHue);
              FastLED.show();
              startHue++;
              deltaHue++;
              if (startHue > 255) {startHue = 0;}
              if (deltaHue > 255) {deltaHue = 0;}
      }
    }

}
