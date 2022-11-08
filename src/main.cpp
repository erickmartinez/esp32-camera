#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <secrets.h>
#include "esp_wpa2.h"



#define SERIAL_SPEED 115200
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define I2C_SDA 42 
#define I2C_SCL 41
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define LED_GPIO   15
#define PWM1_Ch    0
unsigned long PWM1_Res;

const char* ssid = SECRET_SSID;
const char* password = SECRET_PWD;

char responseBuffer[50];


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiServer wifiServer(3001);
WiFiClient client;

String input;

IPAddress local_IP(192, 168, 4, 4);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 240, 0);

unsigned long currentMillis, previousMillis, interval;
unsigned long totalTimeMS;
float period, pulseWidth, frequency, dc;
unsigned long dutyCycle;
unsigned long PWM1_Freq;
unsigned long startMillis, elapsedMillis, previousOn;
unsigned long periodOn, periodMillis;
char rxChar;
bool triggerFlag, triggerRunning;
bool initLedC;
unsigned long maxDC;


void updateDisplay() {
  char buff[30]; // Buffer big enough for 7-character float  
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.print("IP: ");
  display.println(WiFi.localIP());
  
  snprintf(buff, sizeof(buff), "F: %4.2E Hz\nDC: %5.3f %%", frequency, dc);
  display.println(buff);
  display.display();
}

void updateFreq(unsigned long freq) {
  ledcWrite(PWM1_Ch, 0);
  if (!initLedC) {
    ledcSetup(PWM1_Ch, freq, PWM1_Res);
    initLedC = true;
  } else {
    ledcChangeFrequency(PWM1_Ch, freq, PWM1_Res);
  }
}

void updateDutyCycle() {
  dc = pulseWidth / period;
  if (dc > 1.0) {dc = 1.0;}
  frequency = (period > 0 ) ? (1000.0 / period) : 20E6 ;
  PWM1_Res = (frequency >= 1E4) ? 8 : 12;
  maxDC = (unsigned long) pow(2, (PWM1_Res)) - 1;
  dutyCycle = (unsigned long) (dc * maxDC);
  if (dutyCycle > maxDC) {dutyCycle = maxDC;}
  PWM1_Freq = (unsigned long) frequency;
  if (frequency >= 10) {
    updateFreq(PWM1_Freq);
  } else {
    updateFreq(100);
  }
  periodMillis = (unsigned long) period;
  periodOn = (unsigned long) pulseWidth;
  previousOn = 0;
  snprintf(responseBuffer, sizeof(responseBuffer), "Duty Cycle: %lu\nFreq: %.3E", dutyCycle, 1000.0/period);
  Serial.println(responseBuffer);
  updateDisplay();
}

float getDutyCycle() {
  updateDisplay();
  return pulseWidth / period;
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  // esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID));
  // esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  // esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  // esp_wifi_sta_wpa2_ent_enable();

  WiFi.begin(SECRET_SSID);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to the WiFi network.");
  Serial.println(WiFi.localIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  wifiServer.begin();
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  initWiFi();
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  initWiFi();
  

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(100); // Pause for 0.1 seconds

  // Clear the buffer
  display.clearDisplay();
  interval = 100;
  previousMillis = 0;
  pulseWidth = 50; // milliseconds
  period = 100; // milliseconds
  totalTimeMS = 10000; // milliseconds
  initLedC = false;
  ledcSetup(PWM1_Ch, 100, 12);
  updateDutyCycle();
  ledcAttachPin(LED_GPIO, PWM1_Ch);
  ledcWrite(PWM1_Ch, 0);
  triggerFlag = false;
  triggerRunning = false;
  
}

void loop() {
  currentMillis = millis();
  client = wifiServer.available();
  // if ((unsigned long) (currentMillis - previousMillis) >= interval) {
  //   updateDisplay();
  //   previousMillis = currentMillis;
  // }

  if (triggerFlag) {
    if (PWM1_Freq >= 10) {
      ledcWrite(PWM1_Ch, dutyCycle);
    } else {
      ledcWrite(PWM1_Ch, 0);
    }
    triggerFlag = false;
    triggerRunning = true;
  }

  if (triggerRunning && (elapsedMillis <= totalTimeMS)) {
    if (frequency < 10.0) {
      if ((unsigned int) (currentMillis - previousOn) >= periodMillis) {
        ledcWrite(PWM1_Ch, maxDC);
        previousOn = currentMillis;
      }
      if ((unsigned int) (currentMillis - previousOn) >= periodOn) {
        ledcWrite(PWM1_Ch, 0);
      }
    }
    elapsedMillis = millis() - startMillis;
    //Serial.print("+");
  } else {
    ledcWrite(PWM1_Ch, 0);
    triggerRunning = false; 
    elapsedMillis = 0;
  }

  if (client) {
    if (client.available()>0) {
      input = client.readStringUntil(0x0D);
      rxChar = input[0];
      triggerFlag = false;
      triggerRunning = false;
      elapsedMillis = 0;
      switch (rxChar)
      {
      case 0x69: // i as in ID
        client.print("CAMERA_TRIGGER\n");
        break;
      case 0x77: // w as in pulse Width
        if (input[1] == 0x3F) { // If '?' found
            snprintf(responseBuffer, sizeof(responseBuffer), "%.3E\n", pulseWidth);
            client.print(responseBuffer);
            break;
          }
          pulseWidth = abs((float) atof(input.substring(2).c_str()));
          updateDutyCycle();
          break;
      case 0x70: // p as in period
         if (input[1] == 0x3F) { // If '?' found
            snprintf(responseBuffer, sizeof(responseBuffer), "%.3E\n", period);
            client.print(responseBuffer);
            break;
          }
          period = abs((float) atof(input.substring(2).c_str()));
          updateDutyCycle();
          break;
      case 0x74: // t as in total time
         if (input[1] == 0x3F) { // If '?' found
            sprintf(responseBuffer, "%.3E\n", totalTimeMS/1000);
            client.print(responseBuffer);
            break;
          }
          totalTimeMS = (unsigned long) 1000 * atol(input.substring(2).c_str());
          break;
      case 0x72: //  r as in run
        elapsedMillis = 0;
        startMillis = currentMillis;
        triggerFlag = true;
        break;
      case 0x64: // d as in duty cycle
      updateDutyCycle();
        snprintf(responseBuffer, sizeof(responseBuffer), "%.3E\n", getDutyCycle());
        client.print(responseBuffer);
        break;
      case 0x66: // f as in frequency
        snprintf(responseBuffer, sizeof(responseBuffer), "%.3E\n", frequency);
        client.print(responseBuffer);
        break;
      default:
        client.print("ERR_CMD\n");
        break;
      }
    }
    client.stop();
  }

  if(Serial.available()) {
    input = Serial.readStringUntil(0x0D); // Read until line breaks
    rxChar = input[0];
    switch (rxChar) {
      case 0x0A:
        break;
      case 0x69: // i as in id
        Serial.print("CAMERA_TRIGGER\n");
        break;
      default:
        Serial.print("ERR_CMD\n");
        break;
    }
  }
  
}

