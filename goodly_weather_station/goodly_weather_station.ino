//----------------------------------------------------------------------
// https://github.com/clearwater/SwitecX25
//

//----------------------------------------------------------------------

// FEATURE CONFIGURATION
#define USE_DS18B20 0  // Set to 1 to enable DS18B20 outdoor temperature sensor
#define USE_SD 0       // Set to 1 to enable SD card (data logging, plot)

#if USE_SD
#include <SD.h>
#endif
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#if USE_DS18B20
#include <OneWire.h>
#endif
#include <PubSubClient.h> // MQTT library
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
//#include <time.h>
#include <DHT22.h>
#include <RTClib.h>
#if USE_DS18B20
//#include <DS18B20.h>
#include <DallasTemperature.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <EEPROM.h>
#include "src/SwitecX25/SwitecX25.h"
#include "src/NTPClient/NTPClient.h"
#if __has_include("src/wifi_credentials.h")
#include "src/wifi_credentials.h"
#else
const char wifi_ssid[] = "";
const char wifi_password[] = "";
#endif
#include <Preferences.h>

// TIME
WiFiUDP ntpUDP;
// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);
String formattedDate;
String dateStamp;
String timeStamp;

// STATES
enum class DisplayState {
  INIT,
  SENSORS,
  MENU,
  PLOT,
  SUMMARY,
  FORECAST,
  SCREENSAVER
};

DisplayState displayState = DisplayState::MENU;
DisplayState displayStatePrev = DisplayState::INIT;

enum class DialState {
  TEMPERATURE_IN,
  TEMPERATURE_OUT,
  PRESSURE,
  HUMIDITY,
  CENTER,
  OFF
};

//DialState dialStateTop = DialState::TEMPERATURE_OUT;
//DialState dialStateBottom = DialState::HUMIDITY;
DialState dialStateTop = DialState::OFF;
DialState dialStateBottom = DialState::OFF;

// DIAL LIMITS
#define DIAL_TEMPERATURE_MAX 35
#define DIAL_TEMPERATURE_MIN -35
#define DIAL_PRESSURE_MAX 1200
#define DIAL_PRESSURE_MIN 800
#define DIAL_HUMIDITY_MAX 107.14
#define DIAL_HUMIDITY_MIN -7.14

// PINS
#define PIN_SPI_DC 9
#define PIN_SPI_CS_OLED 10
#define PIN_SPI_CS_SD 7
#define PIN_OLED_RESET 8
#define PIN_DHT 6

#define MOTOR1_PIN1 A3
#define MOTOR1_PIN2 A6
#define MOTOR1_PIN3 A6
#define MOTOR1_PIN4 A7
#define MOTOR2_PIN1 A0
#define MOTOR2_PIN2 A1
#define MOTOR2_PIN3 A1
#define MOTOR2_PIN4 A2

#define PIN_ENCODER_CLK 5
#define PIN_ENCODER_DT 4
#define PIN_ENCODER_SW 3

#if USE_DS18B20
#define PIN_ONEWIRE 5  // ESP32 GPIO pin due to OneWire.h implementation
#endif

// USER DEFINES
#define INVALID_NUMBER -9999
#define MESSAGE_SIGN_SEC 0.5
#define MESSAGE_SEC 60
#define DISPLAY_OFF_SEC 60
#define SCREENSAVER_ON_SEC 60
#define DISPLAY_LINES 8   // For font size 1
#define DISPLAY_CHARS 21  // For font size 1
#define SENSOR_READ_INTERV_SEC 10
#define LOG_INTERV_SEC 60 * 10
#define SENSOR_READ_INPUT_DLY_SEC 5
//#define SENSOR_READ_INPUT_DLY_SEC 5
//#define SENSOR_VALUE_ERROR -999
#define ENCODER_DEBOUNCE_MILLIS 100
#define PUSH_DEBOUNCE_MILLIS 100
#if USE_SD
#define DATA_LOG_FILE "/datalog.txt"
#define DATA_LOG_HEADER "Date,Time,TempIn,TempOut,TempPcb,HumidityIn,PressureIn"
#define DIAL_POS_FILE "/dialpos.txt"
#else
#define LOG_BUFFER_RESERVE_BYTES (24UL * 1024UL)
#define LOG_BUFFER_MIN_ENTRIES 128
#define MIN_LOG_INTERV_SEC 10
#define LOG_COMPACT_MIN_SAVINGS 8
#endif

#if !USE_DS18B20
#define SMHI_STATION_ID 71420  // Göteborg A - change to nearest station
#define SMHI_FETCH_INTERV_SEC 3600
#endif

#define TEMP_PLAUS_MIN -60
#define TEMP_PLAUS_MAX 60
#define PRES_PLAUS_MIN 800
#define PRES_PLAUS_MAX 1200
#define HUM_PLAUS_MIN 0
#define HUM_PLAUS_MAX 100

#define NTP_TIMEOUT_SEC 5
#define EEPROM_SIZE 512
#define WIFI_EEPROM_MAGIC 0x57A1
#define WIFI_EEPROM_MAGIC_ADDR 0
#define WIFI_EEPROM_SSID_LEN_ADDR 2
#define WIFI_EEPROM_PASS_LEN_ADDR 3
#define WIFI_EEPROM_DATA_ADDR 4
#define WIFI_MAX_SSID_LEN 32
#define WIFI_MAX_PASS_LEN 63

// ROTARY ENCODER PUSH BUTTON
// detachInterrupt(GPIOPin);
// Interrupt variables
volatile bool encoder_clock_prev;
volatile bool encoder_dt_prev;
volatile bool encoder_push_pressed = false;
volatile int encoder_position = 0;
volatile int encoder_position_raw = 0;
// Loop encoder variables
int encPos = 0;
int encPosPrev = 0;
bool encPushd = false;
bool encPushdPrev = false;
unsigned long lastEncoderInputMillis = 0;
unsigned long lastPushInputMillis = 0;

// USER CONSTANTS
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int charWidth = 6;
const int charHeight = 7;
// y-axis label (characters plus margin)
const int yLabelWidth = charWidth * 4 + 1;


// USER VARIABLES
unsigned long lastInputMillis = millis();
unsigned long lastMessageMillis = 0;
bool messageActive = false;
unsigned long lastMessageSignMillis = 0;
bool messageSignActive = false;
bool messageSignActivePrev = false;
bool plotUseRange = true;
unsigned long lastSensorReadMillis = 0;
unsigned long lastLogMillis;
float temperature_in;
float temperature_out;
float temperature_out_array[] = { INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER };
unsigned int avgSamplesTempOut = 10;
float temperature_pcb;
float pressure;
float humidity_rel;
bool temperature_in_ok = false;
bool temperature_out_ok = false;
bool temperature_pcb_ok = false;
bool pressure_ok = false;
bool humidity_rel_ok = false;
#if !USE_DS18B20
float smhi_temperature = 0.0f;
bool smhi_temperature_ok = false;
unsigned long lastSMHIFetchMillis = 0;
#endif
unsigned int currentMessageLine = 0;
String messages[DISPLAY_LINES];
unsigned int storedDialPosTop;
unsigned int storedDialPosBottom;
Preferences motorPosPrefs;
Preferences dialStatePrefs;
Preferences plotStatePrefs;
float plotHours[] = { 1.0, 12, 24, 24 * 7, 24 * 30, 24 * 365, 24 * 365 * 2, 24 * 365 * 5, 24 * 365 * 10, 24 * 365 * 20, 24 * 365 * 50, 24 * 365 * 100 };
unsigned int plotHoursIdx = 2;
bool newLogData;
unsigned int effectiveLogIntervSec = LOG_INTERV_SEC;
#if !USE_SD
struct LogEntry;
unsigned int plotRangeIntervalsSec[sizeof(plotHours) / sizeof(plotHours[0])] = { 0 };
struct LogEntry {
  unsigned long epoch;
  float tempIn;
  float tempOut;
  float tempPcb;
  float hum;
  float pres;
  float tempInMin;
  float tempInMax;
  float tempOutMin;
  float tempOutMax;
  float tempPcbMin;
  float tempPcbMax;
  float humMin;
  float humMax;
  float presMin;
  float presMax;
  uint16_t tempInSamples;
  uint16_t tempOutSamples;
  uint16_t tempPcbSamples;
  uint16_t humSamples;
  uint16_t presSamples;
  bool tempInOk;
  bool tempOutOk;
  bool tempPcbOk;
  bool humOk;
  bool presOk;
};
LogEntry *logBuffer = nullptr;
unsigned int logBufferCapacity = 0;
unsigned int logBufferHead = 0;
unsigned int logBufferCount = 0;
#endif
bool updatePlot;
bool updateAxes;
bool newSensorReadings = false;
bool logging = true;
bool zeroDialsAtStartup = false;
bool timeOk = false;
bool clearLogPressed = false;

// Menu
unsigned int menuSelIdx = 0;
const char *menus[] = { "Main", "Plot", "Settings" };
enum class MenuState { MAIN,
                       PLOT,
                       SETTINGS };
MenuState menuState = MenuState::MAIN;
const unsigned int menusLengths[] = { 5, 6, 6 };
const char *menuContents[][6] = {
  { "Plot",
    "Summary",
    "Sensors",
    "Settings",
    "Forecast" },
  { "Indoor temperature",
    "Outdoor temperature",
    "Pressure",
    "Humidity",
    "PCB temperature",
    "Return" },
  { "Top dial",
    "Bottom dial",
    "Center dials",
    "Logging",
    "Clear log",
    "Return" }
};

const char *plotVars[] = {
  "TempIn",
  "TempOut",
  "PressureIn",
  "HumidityIn",
  "TempPcb"
};
const char *plotTitle[] = {
  "Temp\nIn",
  "Temp\nOut",
  "Pres",
  "Hum",
  "Temp\nPCB"
};
int plotVarsIdx = 0;


// DHT22 TEMP AND HUMIDITY SENSOR
DHT22 dht22(PIN_DHT);
float dht_temperature;
float dht_humidity_rel;
bool dht_temperature_ok = false;
bool dht_humidity_rel_ok = false;

// BMP180 PRESSURE (AND TEMP) SENSOR (DEFAULT I2C PINS A4, A5)
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
float bmp_temperature;
float bmp_pressure;
bool bmp_temperature_ok = false;
bool bmp_pressure_ok = false;

// DS18B20 1wire temp sensor
#if USE_DS18B20
//DS18B20 ds(PIN_ONEWIRE);
//uint8_t ds_address[] = { 40, 250, 31, 218, 4, 0, 0, 52 };
//uint8_t ds_selected;
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds(&oneWire);
float ds_temperature;
bool ds_temperature_ok = false;
#endif

// STEPPER MOTOR DEFINES
// standard X25.168 range 315 degrees at 1/3 degree steps
// #define STEPS (315*3)
// #define STEPS (360*3)
// #define DIAL_RANGE_DEG 216 // Limited to front markings
#define DIAL_RANGE_DEG 252  // Limited to front markings plus one more
#define DIAL_RANGE_STEPS (DIAL_RANGE_DEG * 3)
#define DIAL_CAL_STEPS 5
// SwitecX25 motor1(STEPS,9,7,5,3);
SwitecX25 *motor1;
SwitecX25 *motor2;

// OLED DISPLAY DEFINES
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define SCREEN_MARGIN_TOP 5
#define SCREEN_MARGIN_LEFT 1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         &SPI, PIN_SPI_DC, PIN_OLED_RESET, PIN_SPI_CS_OLED);
bool displayOn = true;
bool displayOnPrev = false;

// Screensaver variables
bool useScreensaver = true;
bool useDisplayOff = false;
unsigned long lastMarqueeUpdate = 0;
unsigned int marqueeSpeed = 5000; // ms between marquee updates
String screensaverText = "Goodly Weather Station";
bool screensaverShowOutdoor = false;
int marqueeX = 0;
int marqueeY = 0;
int marqueeDir = 1;
int marqueeTextWidth = 0;

#define PRESSURE_HISTORY_SIZE 24
#define PRESSURE_HISTORY_DIFFERENCE 1  // Difference in pressure to consider a new reading
#define PRESSURE_HISTORY_INTERVAL_MINUTES 60 // 60 minutes normally?
float pressureHistory[PRESSURE_HISTORY_SIZE] = {INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER, INVALID_NUMBER};
unsigned long pressureHistoryMillis = 0;

unsigned long forecastMinutes = 0;
int forecastPresGradDir = 0;

const char* mqtt_server = "192.168.1.184";
const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {
  initSerial();
  initDisplay();
  initEncoder();
  initBmp();
#if USE_DS18B20
  initDs();
#endif
#if USE_SD
  initSd();
  //removeDataLogFile();
  initDataLogFile();
#endif
  initWifi();
#if !USE_SD
  initAdaptiveLogIntervals();
#endif
  //initTime();
  // initDht();
  readDialStates();
  readPlotSettings();
  initSteppers();
#if !USE_SD
  initLogBuffer();
#endif
  readSensors();
#if !USE_DS18B20
  fetchSMHITemperature();
  lastSMHIFetchMillis = millis();
#endif
  arbitrateSensorReadings();
  getTimeStamp();
  logData();  // Log initial boot data point so plots aren't empty
  updateDialPos(motor1, dialStateTop);
  updateDialPos(motor2, dialStateBottom);

  encPosPrev = encoder_position;
  encPushdPrev = true;
  lastInputMillis = millis();
  lastLogMillis = millis();  // Delay first periodic log (sensor startup?)
  
  // Keep startup messages visible for MESSAGE_SEC seconds
  lastMessageMillis = millis();
  messageActive = true;

  // Initialize MQTT client
  mqttClient.setServer(mqtt_server, mqtt_port);
}

void loop() {

  getEncoder();

  if (encPos != encPosPrev || (encPushd && !encPushdPrev)) {
    lastInputMillis = millis();
    //Serial.println("Input");
  }

  if (useDisplayOff) {
    if (millis() - lastInputMillis < DISPLAY_OFF_SEC * 1000) {
      setDisplayOn();
    } else if (useDisplayOff) {
      setDisplayOff();
      displayState = DisplayState::SUMMARY;
      clearLogPressed = false;
    }
  }

  if (useScreensaver && displayOn) {
    if (millis() - lastInputMillis > SCREENSAVER_ON_SEC * 1000) {
      setDisplayState(DisplayState::SCREENSAVER);
      display.ssd1306_command(0x81); // SSD1306_SETCONTRAST command
      display.ssd1306_command(1);    // Lower value for dimmer display (0-255)
    } else if (displayState == DisplayState::SCREENSAVER) {
      setDisplayState(DisplayState::SUMMARY);
      display.ssd1306_command(0x81);
      display.ssd1306_command(255); // Lower value for dimmer display (0-255)
      clearLogPressed = false;
    }
  }

  if ((displayOn && !displayOnPrev) || 
      (displayState != DisplayState::SCREENSAVER &&
      displayStatePrev == DisplayState::SCREENSAVER)) {
    // Ignore encoder if display was off or screensaver was active
    encPosPrev = encPos;
  }

  if (millis() - lastMessageMillis > MESSAGE_SEC * 1000) {
    messageActive = false;
  }

  if (displayOn) {

    if (!messageActive) {
      switch (displayState) {

        case DisplayState::MENU:
          if (displayStatePrev != displayState) {
            menuSelIdx = 0;
            displayStatePrev = displayState;
            clearLogPressed = false;
          }

          displayMenu(menuContents[(unsigned int)menuState], menusLengths[(unsigned int)menuState], menuSelIdx);

          if (encPos > encPosPrev && menuSelIdx < menusLengths[(unsigned int)menuState] - 1) {
            menuSelIdx++;
          } else if (encPos < encPosPrev && menuSelIdx > 0) {
            menuSelIdx--;
          } else if (encPushd && !encPushdPrev) {
            switch (menuState) {
              case MenuState::MAIN:
                if (isMenuSelection("Plot")) {
                  menuState = MenuState::PLOT;
                } else if (isMenuSelection("Sensors")) {
                  setDisplayState(DisplayState::SENSORS);
                } else if (isMenuSelection("Settings")) {
                  menuState = MenuState::SETTINGS;
                  menuSelIdx = 0;
                } else if (isMenuSelection("Summary")) {
                  setDisplayState(DisplayState::SUMMARY);
                } else if (isMenuSelection("Forecast")) {
                  setDisplayState(DisplayState::FORECAST);
                }
                break;
              case MenuState::PLOT:
                if (isMenuSelection("Return")) {
                  menuState = MenuState::MAIN;
                } else {
                  plotVarsIdx = menuSelIdx;
                  plotUseRange = true;
                  setDisplayState(DisplayState::PLOT);
                  menuState = MenuState::MAIN;
                }
                menuSelIdx = 0;
                break;
              case MenuState::SETTINGS:
                if (isMenuSelection("Clear log")) {
                  if (clearLogPressed) {
                    //removeDataLogFile();
                    //initDataLogFile();
                    menuState = MenuState::MAIN;
                    menuSelIdx = 0;
                    clearLogPressed = false;
                  } else {
                    clearLogPressed = true;
                  }
                } else if (isMenuSelection("Return")) {
                  menuState = MenuState::MAIN;
                  clearLogPressed = false;
                  menuSelIdx = 0;
                } else if (isMenuSelection("Logging")) {
                  logging = !logging;
                } else if (isMenuSelection("Top dial")) {
                  if (dialStateTop == DialState::OFF) {
                    dialStateTop = (DialState)0;
                  } else {
                    dialStateTop = (DialState)((int)dialStateTop + 1);
                  }
                  saveDialStates();
                } else if (isMenuSelection("Bottom dial")) {
                  if (dialStateBottom == DialState::OFF) {
                    dialStateBottom = (DialState)0;
                  } else {
                    dialStateBottom = (DialState)((int)dialStateBottom + 1);
                  }
                  saveDialStates();
                } else if (isMenuSelection("Center dials")) {
                    zeroDials();
                    setDisplayState(DisplayState::MENU);
                  }
                break;
            }
          }
          break;

        case DisplayState::SENSORS:
          displaySensorReadings();
          if (encPushd && !encPushdPrev) {
            setDisplayState(DisplayState::MENU);
          }
          break;
        case DisplayState::SUMMARY:
          displaySummary();
          if (encPushd && !encPushdPrev) {
            setDisplayState(DisplayState::MENU);
          }
          break;
        case DisplayState::FORECAST:
          displayForecast();
          if (encPushd && !encPushdPrev) {
            setDisplayState(DisplayState::MENU);
          }
          break;

        case DisplayState::PLOT:
          if (displayStatePrev != displayState || (displayOn && !displayOnPrev)) {
            updateAxes = true;
            updatePlot = true;
            displayStatePrev = displayState;
          }
          if (encPosPrev != encPos) {
            if (encPosPrev < encPos) {
              incPlotTime();
            } else {
              decPlotTime();
            }
            updateAxes = true;
            updatePlot = true;
          }
          if (messageSignActive && millis() - lastMessageSignMillis > MESSAGE_SIGN_SEC * 1000) {
            messageSignActive = false;
            updateAxes = true;
            updatePlot = true;
          }
          if (newLogData && !messageSignActive) {
            updatePlot = true;
            newLogData = false;
          }
          if (updateAxes) {
            plotAxes(plotHours[plotHoursIdx], plotTitle[plotVarsIdx]);
            updateAxes = false;
          }
          if (updatePlot) {
            plotData(plotVars[plotVarsIdx], plotHours[plotHoursIdx], dateStamp, timeStamp, plotTitle[plotVarsIdx]);
            updatePlot = false;
          }
          if (displayStatePrev != displayState) {
            //displayMessageSignHours(plotHours[plotHoursIdx]);
            displayStatePrev = displayState;
          }
          if (encPushd && !encPushdPrev) {
            if (plotUseRange) {
              plotUseRange = false;
              updateAxes = true;
              updatePlot = true;
            } else {
              setDisplayState(DisplayState::MENU);
              menuState = MenuState::MAIN;
            }
          }
          break;

        case DisplayState::SCREENSAVER:
          if (millis() - lastMarqueeUpdate > marqueeSpeed) {
            displayScreensaver();
            lastMarqueeUpdate = millis();
          }
          // Exit screensaver on encoder input
          if (encPos != encPosPrev || (encPushd && !encPushdPrev)) {
            setDisplayState(DisplayState::MENU);
            lastInputMillis = millis();
          }
          break;
      }
    } else if (encPushd && !encPushdPrev) {
      messageActive = false;
    }
    displayOnPrev = displayOn;
  }

#if !USE_DS18B20
  if (WiFi.status() == WL_CONNECTED && millis() - lastSMHIFetchMillis > SMHI_FETCH_INTERV_SEC * 1000UL) {
    fetchSMHITemperature();
    lastSMHIFetchMillis = millis();
  }
#endif

  if ((millis() - lastSensorReadMillis > SENSOR_READ_INTERV_SEC * 1000)
      && (millis() - lastInputMillis > SENSOR_READ_INPUT_DLY_SEC * 1000)) {
    readSensors();
    arbitrateSensorReadings();
    forecastWeather();
    getTimeStamp();
    publishSensorDataToMQTT();
#if USE_SD
    if (timeOk && millis() - lastLogMillis > (unsigned long)effectiveLogIntervSec * 1000UL && logging) {
      logData();
      lastLogMillis = millis();
      //catFileSerial(DATA_LOG_FILE);
    }
#else
    if (millis() - lastLogMillis > (unsigned long)effectiveLogIntervSec * 1000UL && logging) {
      logData();
      lastLogMillis = millis();
    }
#endif
  }

  updateDialPos(motor1, dialStateTop);
  updateDialPos(motor2, dialStateBottom);

  motor1->update();
  motor2->update();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
  }
}

// INIT FUNCTIONS

void initDisplay() {
  // OLED DISPLAY INIT
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  digitalWrite(PIN_SPI_CS_SD, HIGH);  // set other SPI devices' CS high
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;  // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  // display.display();
  // delay(1000);
  // testdrawcircle();
  // Clear the buffer
  display.clearDisplay();
  display.display();
  // Init text, enough to keep here?
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font
}

#if USE_SD
void initDataLogFile() {
  if (!SD.exists(DATA_LOG_FILE)) {
    File dataFile = SD.open(DATA_LOG_FILE, FILE_WRITE);
    if (dataFile) {
      dataFile.print(DATA_LOG_HEADER);
      dataFile.print("\r\n");
      dataFile.close();
      displayMessage(String(DATA_LOG_FILE) + " created");
      //catFileSerial(DATA_LOG_FILE);catFileSerial
    } else {
      displayMessage("Failed to create + " DATA_LOG_FILE);
    }
  }
}

void removeDataLogFile() {
  if (SD.exists(DATA_LOG_FILE)) {
    SD.remove(DATA_LOG_FILE);
    displayMessage(String(DATA_LOG_FILE) + " deleted");
  }
}
#endif

#if USE_SD
void initSd() {
  // SD CARD READER
  digitalWrite(PIN_SPI_CS_OLED, HIGH);
  displayMessage("Initializing SD card");
  if (!SD.begin(PIN_SPI_CS_SD)) {
    displayMessage("SD card failed or not present");
    return;
  } else {
    listFiles(SD.open("/"), 0);
    displayMessage("SD card initialized");
    // File file = SD.open("/wifi.txt", FILE_WRITE);
    // if (file) {
    //   Serial.println("Writing to wifi.txt...");
    //   // Write two lines to the file
    //   file.println("ssid");
    //   file.println("password");
    //   // Close the file
    //   file.close();
    //   Serial.println("Done.");
    // } else {
    //   Serial.println("Error opening file.");
    // }
  }
}
#endif

void initEncoder() {
  // ROTARY ENCODER INTERRUPT
  pinMode(PIN_ENCODER_CLK, INPUT);
  pinMode(PIN_ENCODER_DT, INPUT);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_CLK, isrEncoder, CHANGE);
  //attachInterrupt(PIN_ENCODER_DT, isrEncoder, CHANGE);
  attachInterrupt(PIN_ENCODER_SW, isrEncoderPush, RISING);
}

void initBmp() {
  // BMP180 INIT
  /* Initialise the sensor */
  displayMessage("Initializing BMP085");
  if (!bmp.begin()) {
    /* There was a problem detecting the BMP085 ... check your connections */
    displayMessage("No BMP085 detected, check I2C");
    // while(1);
  } else {
    bmp_temperature_ok = true;
    bmp_pressure_ok = true;
  }
}

void initSerial() {
  Serial.begin(115200);
  delay(1000);
  // while (!Serial) {}
}

bool saveWiFiCredentialsToEEPROM(const String &ssid, const String &password) {
  if (ssid.length() == 0) {
    return false;
  }

  String trimmedSsid = ssid;
  String trimmedPassword = password;
  if (trimmedSsid.length() > WIFI_MAX_SSID_LEN) {
    trimmedSsid = trimmedSsid.substring(0, WIFI_MAX_SSID_LEN);
  }
  if (trimmedPassword.length() > WIFI_MAX_PASS_LEN) {
    trimmedPassword = trimmedPassword.substring(0, WIFI_MAX_PASS_LEN);
  }

  if (!EEPROM.begin(EEPROM_SIZE)) {
    return false;
  }

  bool sameAsStored = false;
  uint16_t magic = (uint16_t)EEPROM.read(WIFI_EEPROM_MAGIC_ADDR)
                   | ((uint16_t)EEPROM.read(WIFI_EEPROM_MAGIC_ADDR + 1) << 8);
  if (magic == WIFI_EEPROM_MAGIC) {
    uint8_t storedSsidLen = EEPROM.read(WIFI_EEPROM_SSID_LEN_ADDR);
    uint8_t storedPassLen = EEPROM.read(WIFI_EEPROM_PASS_LEN_ADDR);
    if (storedSsidLen == (uint8_t)trimmedSsid.length() && storedPassLen == (uint8_t)trimmedPassword.length()) {
      sameAsStored = true;
      for (int i = 0; i < storedSsidLen; i++) {
        if ((char)EEPROM.read(WIFI_EEPROM_DATA_ADDR + i) != trimmedSsid[i]) {
          sameAsStored = false;
          break;
        }
      }
      if (sameAsStored) {
        for (int i = 0; i < storedPassLen; i++) {
          if ((char)EEPROM.read(WIFI_EEPROM_DATA_ADDR + WIFI_MAX_SSID_LEN + i) != trimmedPassword[i]) {
            sameAsStored = false;
            break;
          }
        }
      }
    }
  }

  if (sameAsStored) {
    EEPROM.end();
    return true;
  }

  EEPROM.write(WIFI_EEPROM_MAGIC_ADDR, WIFI_EEPROM_MAGIC & 0xFF);
  EEPROM.write(WIFI_EEPROM_MAGIC_ADDR + 1, (WIFI_EEPROM_MAGIC >> 8) & 0xFF);
  EEPROM.write(WIFI_EEPROM_SSID_LEN_ADDR, (uint8_t)trimmedSsid.length());
  EEPROM.write(WIFI_EEPROM_PASS_LEN_ADDR, (uint8_t)trimmedPassword.length());

  for (int i = 0; i < WIFI_MAX_SSID_LEN; i++) {
    char ch = (i < trimmedSsid.length()) ? trimmedSsid[i] : 0;
    EEPROM.write(WIFI_EEPROM_DATA_ADDR + i, (uint8_t)ch);
  }
  for (int i = 0; i < WIFI_MAX_PASS_LEN; i++) {
    char ch = (i < trimmedPassword.length()) ? trimmedPassword[i] : 0;
    EEPROM.write(WIFI_EEPROM_DATA_ADDR + WIFI_MAX_SSID_LEN + i, (uint8_t)ch);
  }

  bool committed = EEPROM.commit();
  EEPROM.end();
  return committed;
}

bool isValidDialState(int state) {
  return state >= (int)DialState::TEMPERATURE_IN && state <= (int)DialState::OFF;
}

void saveDialStates() {
  dialStatePrefs.begin("dialstate", false);
  dialStatePrefs.putUChar("top", (uint8_t)dialStateTop);
  dialStatePrefs.putUChar("bottom", (uint8_t)dialStateBottom);
  dialStatePrefs.end();
}

void readDialStates() {
  dialStatePrefs.begin("dialstate", true);
  int storedTop = dialStatePrefs.getUChar("top", (uint8_t)DialState::TEMPERATURE_IN);
  int storedBottom = dialStatePrefs.getUChar("bottom", (uint8_t)DialState::PRESSURE);

  if (isValidDialState(storedTop)) {
    dialStateTop = (DialState)storedTop;
  }
  if (isValidDialState(storedBottom)) {
    dialStateBottom = (DialState)storedBottom;
  }

  dialStatePrefs.end();
}

void savePlotSettings() {
  plotStatePrefs.begin("plotstate", false);
  plotStatePrefs.putUChar("hoursIdx", (uint8_t)plotHoursIdx);
  plotStatePrefs.end();
}

void readPlotSettings() {
  plotStatePrefs.begin("plotstate", true);
  unsigned int maxIdx = (sizeof(plotHours) / sizeof(plotHours[0])) - 1;
  unsigned int storedIdx = plotStatePrefs.getUChar("hoursIdx", 2);
  if (storedIdx <= maxIdx) {
    plotHoursIdx = storedIdx;
  } else {
    plotHoursIdx = 2;
  }

  plotStatePrefs.end();
}

#if USE_DS18B20
void initDs() {
  // DS18B20 INIT
  displayMessage("Initializing DS18B20");
  //ds_selected = ds.select(ds_address);
  ds.begin();
  ds.setWaitForConversion(true);
  ds.requestTemperatures();
}
#endif

void zeroDials() {
  //motor1->setPosition(DIAL_RANGE_STEPS / 2);
  //motor2->setPosition(DIAL_RANGE_STEPS / 2);

  //motor1->updateBlocking();
  //motor2->updateBlocking();

  displayMessage("Plz center top dial");
  zeroStepper(motor1);

  displayMessage("Plz center bottom dial");
  zeroStepper(motor2);

  storeMotorPos();
}

void initSteppers() {
  // STEPPER MOTOR INIT
  // Setting pin mode in SwitecX25 constructor before setup() does not work
  // Create stepper object here

  displayMessage("Initializing steppers");
  motor1 = new SwitecX25(DIAL_RANGE_STEPS, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_PIN3, MOTOR1_PIN4);
  motor2 = new SwitecX25(DIAL_RANGE_STEPS, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_PIN3, MOTOR2_PIN4);

  // display.clearDisplay();
  // display.setCursor(0, 0);

  readMotorPos();

  motor1->setCurrentStep(storedDialPosTop);
  //displayMessage("Top: " + String(storedDialPosTop));
  motor2->setCurrentStep(storedDialPosBottom);
  //displayMessage("Bottom: " + String(storedDialPosBottom));

  if (zeroDialsAtStartup) {
    zeroDials();
  }
}

void initWifi() {
  displayMessage("Initializing WiFi");

  String selectedSsid = String(wifi_ssid);
  String selectedPassword = String(wifi_password);
  String eepromSsid;
  String eepromPassword;

  if (EEPROM.begin(EEPROM_SIZE)) {
    uint16_t magic = (uint16_t)EEPROM.read(WIFI_EEPROM_MAGIC_ADDR) | ((uint16_t)EEPROM.read(WIFI_EEPROM_MAGIC_ADDR + 1) << 8);
    if (magic == WIFI_EEPROM_MAGIC) {
      uint8_t ssidLen = EEPROM.read(WIFI_EEPROM_SSID_LEN_ADDR);
      uint8_t passLen = EEPROM.read(WIFI_EEPROM_PASS_LEN_ADDR);
      if (ssidLen > 0 && ssidLen <= WIFI_MAX_SSID_LEN && passLen <= WIFI_MAX_PASS_LEN) {
        char ssidBuf[WIFI_MAX_SSID_LEN + 1];
        char passBuf[WIFI_MAX_PASS_LEN + 1];
        for (int i = 0; i < ssidLen; i++) {
          ssidBuf[i] = (char)EEPROM.read(WIFI_EEPROM_DATA_ADDR + i);
        }
        ssidBuf[ssidLen] = '\0';
        for (int i = 0; i < passLen; i++) {
          passBuf[i] = (char)EEPROM.read(WIFI_EEPROM_DATA_ADDR + WIFI_MAX_SSID_LEN + i);
        }
        passBuf[passLen] = '\0';
        eepromSsid = String(ssidBuf);
        eepromPassword = String(passBuf);
        if (eepromSsid.length() > 0) {
          selectedSsid = eepromSsid;
          selectedPassword = eepromPassword;
          Serial.println("[WiFi] Loaded credentials from EEPROM.");
        }
      }
    }
  }

  Serial.println("[WiFi] Press 'w' within 5 seconds to configure WiFi over UART.");
  unsigned long uartStart = millis();
  bool runWizard = false;
  while (millis() - uartStart < 5000) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'w' || c == 'W') {
        runWizard = true;
      }
      while (Serial.available()) {
        Serial.read();
      }
      break;
    }
    delay(10);
  }

  if (runWizard) {
    WiFi.mode(WIFI_STA);
    Serial.println("\n=== WiFi Setup Wizard ===");
    Serial.println("Scanning networks...");
    int networkCount = WiFi.scanNetworks();

    if (networkCount <= 0) {
      Serial.println("No WiFi networks found.");
    } else {
      for (int i = 0; i < networkCount; i++) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" (RSSI ");
        Serial.print(WiFi.RSSI(i));
        Serial.print(" dBm, ");
        Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "secured");
        Serial.println(")");
      }

      Serial.println("Enter network number and press Enter:");
      Serial.setTimeout(60000);
      String line = Serial.readStringUntil('\n');
      line.trim();
      int selectedIdx = line.toInt() - 1;

      if (selectedIdx >= 0 && selectedIdx < networkCount) {
        selectedSsid = WiFi.SSID(selectedIdx);
        if (WiFi.encryptionType(selectedIdx) == WIFI_AUTH_OPEN) {
          selectedPassword = "";
        } else {
          Serial.println("Enter password and press Enter:");
          String pass = Serial.readStringUntil('\n');
          pass.trim();
          selectedPassword = pass;
        }

        if (selectedSsid.length() > WIFI_MAX_SSID_LEN) {
          selectedSsid = selectedSsid.substring(0, WIFI_MAX_SSID_LEN);
        }
        if (selectedPassword.length() > WIFI_MAX_PASS_LEN) {
          selectedPassword = selectedPassword.substring(0, WIFI_MAX_PASS_LEN);
        }
        if (saveWiFiCredentialsToEEPROM(selectedSsid, selectedPassword)) {
          Serial.println("WiFi credentials saved to EEPROM.");
        } else {
          Serial.println("Failed to save WiFi credentials to EEPROM.");
        }
      } else {
        Serial.println("Invalid selection. Using existing credentials.");
      }
    }
  }

  if (selectedSsid.length() == 0) {
    Serial.println("[WiFi] No credentials loaded. Use UART wizard with 'w'.");
    displayMessage("No WiFi credentials");
    return;
  }

  if (saveWiFiCredentialsToEEPROM(selectedSsid, selectedPassword)) {
    Serial.println("[WiFi] Credentials stored in EEPROM.");
  } else {
    Serial.println("[WiFi] Failed to store credentials in EEPROM.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(selectedSsid.c_str(), selectedPassword.c_str());
  // Will try for about 10 seconds (20x 500ms)
  int tryDelay = 500;
  int numberOfTries = 20;

  // Wait for the WiFi event
  while (true) {
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("[WiFi] SSID not found");
        break;
      case WL_CONNECT_FAILED:
        Serial.print("[WiFi] Failed - WiFi not connected! Reason: ");
        return;
        break;
      case WL_CONNECTION_LOST:
        Serial.println("[WiFi] Connection was lost");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("[WiFi] Scan is completed");
        break;
      case WL_DISCONNECTED:
        Serial.println("[WiFi] WiFi is disconnected");
        break;
      case WL_CONNECTED:
        Serial.println("[WiFi] WiFi is connected!");
        Serial.print("[WiFi] IP address: ");
        Serial.println(WiFi.localIP());
        return;
        break;
      default:
        Serial.print("[WiFi] WiFi Status: ");
        Serial.println(WiFi.status());
        break;
    }
    delay(tryDelay);

    if (numberOfTries <= 0) {
      Serial.println("[WiFi] Failed to connect to WiFi!");
      // Use disconnect function to force stop trying to connect
      WiFi.disconnect();
      return;
    } else {
      numberOfTries--;
    }
  }
}

void initTime() {
  displayMessage("Initializing NTP time");
  timeClient.begin();
  getTimeStamp();
  displayMessage(dateStamp + " " + timeStamp);
}

// INTERRUPT FUNCTIONS

void isrEncoder() {
  bool encoder_clock = digitalRead(PIN_ENCODER_CLK);
  if (encoder_clock != encoder_clock_prev) {
    bool encoder_dt = digitalRead(PIN_ENCODER_DT);
    if (encoder_dt != encoder_clock) {
      encoder_position_raw++;
    } else {
      encoder_position_raw--;
    }
    encoder_clock_prev = encoder_clock;
    encoder_position = encoder_position_raw / 2;
  }
}


// IRAM_ATTR?
void isrEncoderPush() {
  encoder_push_pressed = true;
}

// OTHER FUNCTIONS

bool isMenuSelection(const char *text) {
  if (strcmp(menuContents[(unsigned int)menuState][menuSelIdx], text) == 0) {
    return true;
  } else {
    return false;
  }
}

#if !USE_DS18B20
void fetchSMHITemperature() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();  // Public open data, no cert pinning needed
  client.setTimeout(8000);

  HTTPClient https;
  String url = "https://opendata-download-metobs.smhi.se/api/version/latest/parameter/1/station/"
               + String(SMHI_STATION_ID) + "/period/latest-hour/data.json";

  if (!https.begin(client, url)) {
    Serial.println("[SMHI] Failed to begin HTTPS");
    return;
  }

  https.setTimeout(8000);
  int httpCode = https.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("[SMHI] HTTP error: " + String(httpCode));
    https.end();
    return;
  }

  String body = https.getString();
  https.end();

  // Parse temperature from: "value":[{"date":...,"value":"21.5","quality":"G"}]
  int arrayStart = body.indexOf("\"value\":[");
  if (arrayStart < 0) {
    Serial.println("[SMHI] No value array in response");
    smhi_temperature_ok = false;
    return;
  }

  // Find the "value":"..." key inside the array element
  int valueStart = body.indexOf("\"value\":\"", arrayStart + 9);
  if (valueStart < 0) {
    Serial.println("[SMHI] No value key in array");
    smhi_temperature_ok = false;
    return;
  }

  valueStart += 9;  // skip past "value":"
  int valueEnd = body.indexOf("\"", valueStart);
  if (valueEnd <= valueStart) {
    Serial.println("[SMHI] Could not delimit temperature string");
    smhi_temperature_ok = false;
    return;
  }

  String tempStr = body.substring(valueStart, valueEnd);
  float temp = tempStr.toFloat();

  if (temp > TEMP_PLAUS_MIN && temp < TEMP_PLAUS_MAX) {
    smhi_temperature = temp;
    smhi_temperature_ok = true;
    Serial.println("[SMHI] Outdoor temp: " + String(smhi_temperature, 1) + " C");
  } else {
    smhi_temperature_ok = false;
    Serial.println("[SMHI] Value out of plausible range: " + tempStr);
  }
}
#endif

void arbitrateSensorReadings() {
  // Indoor temp
  if (dht_temperature > TEMP_PLAUS_MIN && dht_temperature < TEMP_PLAUS_MAX && dht_temperature_ok) {
    temperature_in = dht_temperature;
    temperature_in_ok = true;
  } else {
    temperature_in_ok = false;
  }

  // Use rolling average for outdoor temperature
  for (int i = 0; i < avgSamplesTempOut - 1; ++i) {
    temperature_out_array[i] = temperature_out_array[i + 1];
  }
#if USE_DS18B20
  if (ds_temperature > TEMP_PLAUS_MIN && ds_temperature < TEMP_PLAUS_MAX && ds_temperature_ok) {
    temperature_out_array[avgSamplesTempOut - 1] = ds_temperature;
  } else {
    temperature_out_array[avgSamplesTempOut - 1] = INVALID_NUMBER;
  }
#else
  if (smhi_temperature_ok && smhi_temperature > TEMP_PLAUS_MIN && smhi_temperature < TEMP_PLAUS_MAX) {
    temperature_out_array[avgSamplesTempOut - 1] = smhi_temperature;
  } else {
    temperature_out_array[avgSamplesTempOut - 1] = INVALID_NUMBER;
  }
#endif
  int avgSamples = 0;
  float sum = 0;
  for (int i = 0; i < avgSamplesTempOut; ++i) {
    if (temperature_out_array[i] != INVALID_NUMBER) {
      sum += temperature_out_array[i];
      avgSamples++;
    }
  }
  if (avgSamples > 0) {
    temperature_out = sum / avgSamples;
    temperature_out_ok = true;
  } else {
    temperature_out_ok = false;
  }
  // DEBUG
  // Serial.println("array:");
  // for (int i = 0; i < avgSamplesTempOut; ++i) {
  //   Serial.println(temperature_out_array[i]);
  // }

  // PCB temp
  if (bmp_temperature > TEMP_PLAUS_MIN && bmp_temperature < TEMP_PLAUS_MAX && bmp_temperature_ok) {
    temperature_pcb = bmp_temperature;
    temperature_pcb_ok = true;
  } else {
    temperature_pcb_ok = false;
  }

  // Pressure
  if (bmp_pressure > PRES_PLAUS_MIN && bmp_pressure < PRES_PLAUS_MAX && bmp_pressure_ok) {
    pressure = bmp_pressure;
    pressure_ok = true;
  } else {
    pressure_ok = false;
  }

  // Store pressure in circular buffer
  if (millis() - pressureHistoryMillis > PRESSURE_HISTORY_INTERVAL_MINUTES * 60 * 1000) {
    pressureHistoryMillis = millis();
    // Shift the pressure history
    for (int i = 0; i < PRESSURE_HISTORY_SIZE - 1; i++) {
      pressureHistory[i] = pressureHistory[i + 1];
    }
    // Add the new pressure reading to the end of the history
    // If pressure is valid, store it, otherwise store INVALID_NUMBER
    // This way we always have the latest reading at the end of the array
    // and the oldest reading at the start of the array
    if (pressure_ok) {
      pressureHistory[PRESSURE_HISTORY_SIZE - 1] = pressure;
    } else {
      pressureHistory[PRESSURE_HISTORY_SIZE - 1] = INVALID_NUMBER;
    }
  }

  // Humidity
  if (dht_humidity_rel > HUM_PLAUS_MIN && dht_humidity_rel < HUM_PLAUS_MAX && dht_humidity_rel_ok) {
    humidity_rel = dht_humidity_rel;
    humidity_rel_ok = true;
  } else {
    humidity_rel_ok = false;
  }
  newSensorReadings = true;
}

void zeroStepper(SwitecX25 *motor) {
  int encoder_position_prev = encoder_position;
  encoder_push_pressed = false;
  while (!encoder_push_pressed) {
    if (encoder_position > encoder_position_prev) {
      motor->setCurrentStep(DIAL_RANGE_STEPS / 2);
      motor->setPosition(DIAL_RANGE_STEPS / 2 + DIAL_CAL_STEPS);
      encoder_position_prev = encoder_position;
    } else if (encoder_position < encoder_position_prev) {
      motor->setCurrentStep(DIAL_RANGE_STEPS / 2);
      motor->setPosition(DIAL_RANGE_STEPS / 2 - DIAL_CAL_STEPS);
      encoder_position_prev = encoder_position;
    }
    motor->update();
  }
  encoder_push_pressed = false;
  motor->setCurrentStep(DIAL_RANGE_STEPS / 2);
}

void updateDialPos(SwitecX25 *motor, DialState state) {
  unsigned int position;
  switch (state) {
    case DialState::TEMPERATURE_IN:
      position = valToDialPos(temperature_in, DIAL_TEMPERATURE_MIN, DIAL_TEMPERATURE_MAX);
      motor->setPosition(position);
      break;
    case DialState::TEMPERATURE_OUT:
      position = valToDialPos(temperature_out, DIAL_TEMPERATURE_MIN, DIAL_TEMPERATURE_MAX);
      motor->setPosition(position);
      break;
    case DialState::PRESSURE:
      position = valToDialPos(pressure, DIAL_PRESSURE_MIN, DIAL_PRESSURE_MAX);
      motor->setPosition(position);
      break;
    case DialState::HUMIDITY:
      position = valToDialPos(humidity_rel, DIAL_HUMIDITY_MIN, DIAL_HUMIDITY_MAX);
      motor->setPosition(position);
      break;
    case DialState::CENTER:
      position = valToDialPos(0, -1, 1);
      motor->setPosition(position);
      break;
  }
  storeMotorPos();
}

unsigned int valToDialPos(float val, float min, float max) {
  return map_float(val, min, max, 0, DIAL_RANGE_STEPS);
}

void getEncoder() {

  encPosPrev = encPos;
  encPos = encoder_position;

  encPushdPrev = encPushd;
  if (encoder_push_pressed) {
    unsigned long current_millis = millis();
    if (current_millis - lastPushInputMillis > PUSH_DEBOUNCE_MILLIS) {
      encPushd = true;
      lastPushInputMillis = current_millis;
    } else {
      encPushd = false;
    }
    encoder_push_pressed = false;
  } else {
    encPushd = false;
  }
}

void displayEncoderReadings() {
  bool CLK = true;
  bool DT = true;
  bool SW = true;
  display.clearDisplay();
  display.println("CLK, DT, SW:");
  display.display();
  delay(1000);
  for (;;) {
    CLK = digitalRead(PIN_ENCODER_CLK);
    DT = digitalRead(PIN_ENCODER_DT);
    SW = digitalRead(PIN_ENCODER_SW);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Hej");
    display.println(CLK);
    display.println(DT);
    display.println(SW);
    display.display();
  }
}

void setDisplayOff() {
  displayOnPrev = displayOn;
  displayOn = false;
  display.clearDisplay();
  display.display();
}

void setDisplayOn() {
  displayOnPrev = displayOn;
  displayOn = true;
}

void setDisplayState(DisplayState newState) {
  displayStatePrev = displayState;
  displayState = newState;
  if (newState == DisplayState::SCREENSAVER && displayStatePrev != DisplayState::SCREENSAVER) {
    screensaverShowOutdoor = false;
  }
}

void readBmp() {
  sensors_event_t event;
  bmp.getEvent(&event);
  bmp_pressure = event.pressure;
  bmp.getTemperature(&bmp_temperature);
}

#if USE_DS18B20
void readDs() {
  //ds_temperature = ds.getTempC();
  ds.requestTemperatures();
  ds_temperature = ds.getTempCByIndex(0);
  if (ds_temperature == DEVICE_DISCONNECTED_C) {
    displayMessage("DS18B20 disconnected");
    ds_temperature_ok = false;
  } else {
    ds_temperature_ok = true;
  }
}
#endif

void readDht() {
  dht_temperature = dht22.getTemperature();
  delay(50);  // For stable readings
  dht_humidity_rel = dht22.getHumidity();
  if (dht22.getLastError() != dht22.OK) {
    dht_temperature_ok = false;
    dht_humidity_rel_ok = false;
  } else {
    dht_temperature_ok = true;
    dht_humidity_rel_ok = true;
  }
}

void readSensors() {
  readDht();
  readBmp();
#if USE_DS18B20
  readDs();
#endif
  lastSensorReadMillis = millis();
}

void displaySensorReadings() {
  display.clearDisplay();
  display.setCursor(0, 0);  // Start at top-left corner

  display.print("\n");

  display.print("Temp bmp: ");
  display.print(bmp_temperature);
  display.println(" C");

  display.print("Pressure bmp: ");
  display.print(bmp_pressure);
  display.println(" hPa");

  // display.print("\n");

  display.print("Temp dht: ");
  display.print(dht_temperature);
  display.println(" C");

  display.print("Humidity dht: ");
  display.print(dht_humidity_rel);
  display.println(" %");

#if USE_DS18B20
  display.print("Temp ds: ");
  display.print(ds_temperature);
  display.println(" C");
#endif

  display.print("Encoder: ");
  display.print(encPos);

  display.display();
}

void displaySummary() {
  display.clearDisplay();
  display.setCursor(0, 0);  // Start at top-left corner

  display.print("\n");

  display.print("Temp in: ");
  display.print(temperature_in);
  display.println(" C");

  if (temperature_out_ok) {
    display.print("Temp out: ");
    display.print(temperature_out, 1);
#if !USE_DS18B20
    display.println(" C (S)");
#else
    display.println(" C");
#endif
  } else {
    display.print("\n");
  }

  display.print("Pressure: ");
  display.print(pressure);
  display.println(" hPa");

  display.print("Humidity: ");
  display.print(humidity_rel);
  display.println(" %");

  display.println(dateStamp + " " + timeStamp);

  if (logging) {
    display.println("Logging: ON");
  } else {
    display.println("Logging: OFF");
  }

  if (WiFi.status() == WL_CONNECTED) {
    display.print("WiFi: ON");
  } else {
    display.print("WiFi: OFF");
  }

  display.display();
}

#if USE_SD
void listFiles(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // No more files
      break;
    }
    for (unsigned int i = 0; i < numTabs; i++) {
      Serial.print('\t');  // Print tabs for nested files
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      listFiles(entry, numTabs + 1);  // Recursively list subdirectories
    } else {
      // Files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
#endif

#if USE_SD
void catFileSerial(const char *path) {
  File file = SD.open(path, FILE_READ);
  if (file) {
    Serial.println("Contents of " + String(path) + ":");
    Serial.println("==========");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    Serial.println("==========");
    Serial.println("End of file.");
  } else {
    Serial.println("Error opening " + String(path));
  }
}
#endif

// Function to get date and time from NTPClient
void getTimeStamp() {
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long startTime = millis();
    bool timeout = false;
    while (!timeClient.update()) {
      if (millis() - startTime > NTP_TIMEOUT_SEC * 1000) {
        timeout = true;
        break;
      }
      timeClient.forceUpdate();
    }
    if (timeout) {
      //displayMessage("NTP update failed");
      timeOk = false;
    } else if (timeClient.isTimeSet()) {
      timeOk = true;
    }

    // The formattedDate comes with the following format:
    // 2018-05-28T16:00:13Z
    // We need to extract date and time
    formattedDate = timeClient.getFormattedDate();
    // Serial.println(formattedDate);

    // Extract date
    int splitT = formattedDate.indexOf("T");
    dateStamp = formattedDate.substring(0, splitT);
    // Extract time
    timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  } else {
    timeOk = false;
    if (logging) {
      displayMessage("No WiFi, no timestamp");
    }
  }
}

#if USE_SD
void logData() {
  // "Date,Time,TempIn,TempOut,TempPcb,HumidityIn,PressureIn"
  File file = SD.open(DATA_LOG_FILE, FILE_WRITE);
  if (file) {

    String tempIn = (temperature_in_ok) ? String(temperature_in, 1) : "NaN";
    String tempOut = (temperature_out_ok) ? String(temperature_out, 1) : "NaN";
    String tempPcb = (temperature_pcb_ok) ? String(temperature_pcb, 1) : "NaN";
    String hum = (humidity_rel_ok) ? String(humidity_rel, 1) : "NaN";
    String pres = (pressure_ok) ? String(pressure, 1) : "NaN";

    String log =
      dateStamp + "," + timeStamp + "," + tempIn + "," + tempOut + "," + tempPcb + "," + hum + "," + pres + "\r\n";

    file.seek(file.size());
    file.println(log);
    Serial.println("Log string: " + log);
    file.close();
    newLogData = true;
  } else {
    displayMessage("Error writing " + String(DATA_LOG_FILE));
  }
}
#endif

#if !USE_SD
unsigned int getPlotDataPoints() {
  int points = SCREEN_WIDTH - yLabelWidth - 1;
  if (points < 1) {
    return 1;
  }
  return (unsigned int)points;
}

unsigned int secondsPerPlotColumn(float hours) {
  unsigned int points = getPlotDataPoints();
  unsigned long seconds = (unsigned long)(hours * 3600.0f + 0.5f);
  unsigned int interval = (unsigned int)((seconds + points - 1) / points);
  if (interval < MIN_LOG_INTERV_SEC) {
    interval = MIN_LOG_INTERV_SEC;
  }
  return interval;
}

void initAdaptiveLogIntervals() {
  unsigned int plotHoursCount = sizeof(plotHours) / sizeof(plotHours[0]);
  float shortestHours = plotHours[0];

  for (unsigned int i = 0; i < plotHoursCount; i++) {
    if (plotHours[i] < shortestHours) {
      shortestHours = plotHours[i];
    }
    plotRangeIntervalsSec[i] = secondsPerPlotColumn(plotHours[i]);
  }

  effectiveLogIntervSec = secondsPerPlotColumn(shortestHours);
  Serial.println("Adaptive RAM log interval: " + String(effectiveLogIntervSec) + "s");
}

unsigned int requiredIntervalForAge(unsigned long ageSec) {
  unsigned int plotHoursCount = sizeof(plotHours) / sizeof(plotHours[0]);

  for (unsigned int i = 0; i < plotHoursCount; i++) {
    unsigned long windowSec = (unsigned long)(plotHours[i] * 3600.0f);
    if (ageSec <= windowSec) {
      return plotRangeIntervalsSec[i];
    }
  }

  return plotRangeIntervalsSec[plotHoursCount - 1];
}

void mergeLogMetric(float &dstAvg, float &dstMin, float &dstMax, uint16_t &dstSamples, bool &dstOk,
                    float srcAvg, float srcMin, float srcMax, uint16_t srcSamples, bool srcOk) {
  if (!srcOk || srcSamples == 0) {
    return;
  }

  if (!dstOk || dstSamples == 0) {
    dstAvg = srcAvg;
    dstMin = srcMin;
    dstMax = srcMax;
    dstSamples = srcSamples;
    dstOk = true;
    return;
  }

  unsigned long totalSamples = (unsigned long)dstSamples + (unsigned long)srcSamples;
  if (totalSamples == 0) {
    return;
  }

  dstAvg = (dstAvg * (float)dstSamples + srcAvg * (float)srcSamples) / (float)totalSamples;
  dstMin = min(dstMin, srcMin);
  dstMax = max(dstMax, srcMax);
  if (totalSamples > 65535UL) {
    dstSamples = 65535;
  } else {
    dstSamples = (uint16_t)totalSamples;
  }
  dstOk = true;
}

String formatRetentionText(unsigned long long seconds) {
  unsigned long long minutes = seconds / 60ULL;
  unsigned long long hours = minutes / 60ULL;
  unsigned long long days = hours / 24ULL;
  unsigned long long years = days / 365ULL;

  if (years > 0) {
    return String((unsigned long)years) + "y " + String((unsigned long)(days % 365ULL)) + "d";
  }
  if (days > 0) {
    return String((unsigned long)days) + "d " + String((unsigned long)(hours % 24ULL)) + "h";
  }
  if (hours > 0) {
    return String((unsigned long)hours) + "h " + String((unsigned long)(minutes % 60ULL)) + "m";
  }
  return String((unsigned long)minutes) + "m";
}

unsigned long long estimateSmartRetentionSeconds(unsigned int capacity) {
  if (capacity == 0) {
    return 0;
  }

  unsigned long long remainingEntries = (unsigned long long)capacity;
  unsigned long long coveredSeconds = 0;
  unsigned long long prevWindowSec = 0;
  unsigned int plotHoursCount = sizeof(plotHours) / sizeof(plotHours[0]);

  for (unsigned int i = 0; i < plotHoursCount; i++) {
    unsigned long long windowSec = (unsigned long long)(plotHours[i] * 3600.0f + 0.5f);
    unsigned long long bucketSec = windowSec - prevWindowSec;
    unsigned int intervalSec = plotRangeIntervalsSec[i];
    if (intervalSec == 0) {
      intervalSec = secondsPerPlotColumn(plotHours[i]);
    }

    unsigned long long neededEntries = (bucketSec + (unsigned long long)intervalSec - 1ULL) / (unsigned long long)intervalSec;

    if (remainingEntries >= neededEntries) {
      coveredSeconds = windowSec;
      remainingEntries -= neededEntries;
      prevWindowSec = windowSec;
    } else {
      coveredSeconds = prevWindowSec + remainingEntries * (unsigned long long)intervalSec;
      remainingEntries = 0;
      break;
    }
  }

  if (remainingEntries > 0) {
    unsigned int lastInterval = plotRangeIntervalsSec[plotHoursCount - 1];
    if (lastInterval == 0) {
      lastInterval = secondsPerPlotColumn(plotHours[plotHoursCount - 1]);
    }
    coveredSeconds += remainingEntries * (unsigned long long)lastInterval;
  }

  return coveredSeconds;
}

bool compactRamLog(unsigned long nowEpoch) {
  if (logBuffer == nullptr || logBufferCount < 2 || logBufferCapacity < 2) {
    return false;
  }

  unsigned int beforeCount = logBufferCount;
  size_t usedBytes = (size_t)beforeCount * sizeof(LogEntry);
  LogEntry *linear = (LogEntry *)malloc(usedBytes);
  if (linear == nullptr) {
    return false;
  }

  for (unsigned int i = 0; i < beforeCount; i++) {
    unsigned int idx = (logBufferHead + logBufferCapacity - beforeCount + i) % logBufferCapacity;
    linear[i] = logBuffer[idx];
  }

  unsigned int keepCount = 0;
  bool haveLastKept = false;
  unsigned long lastKeptEpoch = 0;

  for (unsigned int i = 0; i < beforeCount; i++) {
    const LogEntry &entry = linear[i];
    bool keep = false;

    if (i == beforeCount - 1) {
      keep = true;
    } else if (!haveLastKept) {
      keep = true;
    } else if (entry.epoch <= lastKeptEpoch) {
      keep = true;
    } else {
      unsigned long ageSec = (nowEpoch > entry.epoch) ? (nowEpoch - entry.epoch) : 0;
      unsigned int minSpacing = requiredIntervalForAge(ageSec);
      keep = (entry.epoch - lastKeptEpoch) >= (unsigned long)minSpacing;
    }

    if (keep) {
      linear[keepCount] = entry;
      keepCount++;
      haveLastKept = true;
      lastKeptEpoch = entry.epoch;
    } else if (keepCount > 0) {
      mergeLogMetric(linear[keepCount - 1].tempIn, linear[keepCount - 1].tempInMin, linear[keepCount - 1].tempInMax, linear[keepCount - 1].tempInSamples, linear[keepCount - 1].tempInOk,
                     entry.tempIn, entry.tempInMin, entry.tempInMax, entry.tempInSamples, entry.tempInOk);
      mergeLogMetric(linear[keepCount - 1].tempOut, linear[keepCount - 1].tempOutMin, linear[keepCount - 1].tempOutMax, linear[keepCount - 1].tempOutSamples, linear[keepCount - 1].tempOutOk,
                     entry.tempOut, entry.tempOutMin, entry.tempOutMax, entry.tempOutSamples, entry.tempOutOk);
      mergeLogMetric(linear[keepCount - 1].tempPcb, linear[keepCount - 1].tempPcbMin, linear[keepCount - 1].tempPcbMax, linear[keepCount - 1].tempPcbSamples, linear[keepCount - 1].tempPcbOk,
                     entry.tempPcb, entry.tempPcbMin, entry.tempPcbMax, entry.tempPcbSamples, entry.tempPcbOk);
      mergeLogMetric(linear[keepCount - 1].hum, linear[keepCount - 1].humMin, linear[keepCount - 1].humMax, linear[keepCount - 1].humSamples, linear[keepCount - 1].humOk,
                     entry.hum, entry.humMin, entry.humMax, entry.humSamples, entry.humOk);
      mergeLogMetric(linear[keepCount - 1].pres, linear[keepCount - 1].presMin, linear[keepCount - 1].presMax, linear[keepCount - 1].presSamples, linear[keepCount - 1].presOk,
                     entry.pres, entry.presMin, entry.presMax, entry.presSamples, entry.presOk);
      if (entry.epoch > linear[keepCount - 1].epoch) {
        linear[keepCount - 1].epoch = entry.epoch;
      }
      lastKeptEpoch = linear[keepCount - 1].epoch;
    }
  }

  if (beforeCount - keepCount < LOG_COMPACT_MIN_SAVINGS && keepCount > 1) {
    mergeLogMetric(linear[1].tempIn, linear[1].tempInMin, linear[1].tempInMax, linear[1].tempInSamples, linear[1].tempInOk,
                   linear[0].tempIn, linear[0].tempInMin, linear[0].tempInMax, linear[0].tempInSamples, linear[0].tempInOk);
    mergeLogMetric(linear[1].tempOut, linear[1].tempOutMin, linear[1].tempOutMax, linear[1].tempOutSamples, linear[1].tempOutOk,
                   linear[0].tempOut, linear[0].tempOutMin, linear[0].tempOutMax, linear[0].tempOutSamples, linear[0].tempOutOk);
    mergeLogMetric(linear[1].tempPcb, linear[1].tempPcbMin, linear[1].tempPcbMax, linear[1].tempPcbSamples, linear[1].tempPcbOk,
                   linear[0].tempPcb, linear[0].tempPcbMin, linear[0].tempPcbMax, linear[0].tempPcbSamples, linear[0].tempPcbOk);
    mergeLogMetric(linear[1].hum, linear[1].humMin, linear[1].humMax, linear[1].humSamples, linear[1].humOk,
                   linear[0].hum, linear[0].humMin, linear[0].humMax, linear[0].humSamples, linear[0].humOk);
    mergeLogMetric(linear[1].pres, linear[1].presMin, linear[1].presMax, linear[1].presSamples, linear[1].presOk,
                   linear[0].pres, linear[0].presMin, linear[0].presMax, linear[0].presSamples, linear[0].presOk);
    if (linear[0].epoch > linear[1].epoch) {
      linear[1].epoch = linear[0].epoch;
    }
    memmove(linear, linear + 1, (size_t)(keepCount - 1) * sizeof(LogEntry));
    keepCount--;
  }

  memset(logBuffer, 0, (size_t)logBufferCapacity * sizeof(LogEntry));
  for (unsigned int i = 0; i < keepCount; i++) {
    logBuffer[i] = linear[i];
  }
  logBufferCount = keepCount;
  logBufferHead = keepCount % logBufferCapacity;

  free(linear);

  if (beforeCount > keepCount) {
    Serial.println("RAM log compacted: " + String(beforeCount) + " -> " + String(keepCount));
    return true;
  }
  return false;
}

bool initLogBuffer() {
  size_t freeHeap = ESP.getFreeHeap();
  size_t maxAllocHeap = ESP.getMaxAllocHeap();
  size_t safeBytes = min(freeHeap, maxAllocHeap);

  if (safeBytes <= LOG_BUFFER_RESERVE_BYTES) {
    displayMessage("RAM log alloc failed");
    return false;
  }

  safeBytes -= LOG_BUFFER_RESERVE_BYTES;
  unsigned int targetEntries = safeBytes / sizeof(LogEntry);

  if (targetEntries < LOG_BUFFER_MIN_ENTRIES) {
    targetEntries = LOG_BUFFER_MIN_ENTRIES;
  }

  while (targetEntries >= LOG_BUFFER_MIN_ENTRIES) {
    logBuffer = (LogEntry *)malloc((size_t)targetEntries * sizeof(LogEntry));
    if (logBuffer != nullptr) {
      memset(logBuffer, 0, (size_t)targetEntries * sizeof(LogEntry));
      logBufferCapacity = targetEntries;
      logBufferHead = 0;
      logBufferCount = 0;
      unsigned long long smartRetentionSec = estimateSmartRetentionSeconds(logBufferCapacity);
      unsigned long long longestPlotRangeSec = (unsigned long long)(plotHours[(sizeof(plotHours) / sizeof(plotHours[0])) - 1] * 3600.0f + 0.5f);
      String retentionText = formatRetentionText(smartRetentionSec);
      displayMessage("Log every " + String(effectiveLogIntervSec) + "s");
      displayMessage("RAM log: " + String(logBufferCapacity) + " entries");
      displayMessage("Smart keeps " + retentionText);
      displayMessage((smartRetentionSec >= longestPlotRangeSec) ? "All plot ranges OK" : "Longest range partial");
      return true;
    }
    targetEntries = (targetEntries * 3) / 4;
  }

  displayMessage("RAM log alloc failed");
  return false;
}

void logData() {
  if (logBuffer == nullptr || logBufferCapacity == 0) {
    return;
  }

  if (logBufferCount >= logBufferCapacity) {
    unsigned long nowEpoch;
    if (timeOk && timeClient.isTimeSet()) {
      nowEpoch = timeClient.getEpochTime();
    } else {
      nowEpoch = millis() / 1000;
    }
    compactRamLog(nowEpoch);
  }

  if (logBufferCount >= logBufferCapacity) {
    logBufferCount = logBufferCapacity - 1;
  }

  LogEntry &entry = logBuffer[logBufferHead];

  if (timeOk && timeClient.isTimeSet()) {
    entry.epoch = timeClient.getEpochTime();
  } else {
    entry.epoch = millis() / 1000;
  }

  entry.tempIn = temperature_in;
  entry.tempOut = temperature_out;
  entry.tempPcb = temperature_pcb;
  entry.hum = humidity_rel;
  entry.pres = pressure;

  entry.tempInMin = temperature_in;
  entry.tempInMax = temperature_in;
  entry.tempOutMin = temperature_out;
  entry.tempOutMax = temperature_out;
  entry.tempPcbMin = temperature_pcb;
  entry.tempPcbMax = temperature_pcb;
  entry.humMin = humidity_rel;
  entry.humMax = humidity_rel;
  entry.presMin = pressure;
  entry.presMax = pressure;

  entry.tempInOk = temperature_in_ok;
  entry.tempOutOk = temperature_out_ok;
  entry.tempPcbOk = temperature_pcb_ok;
  entry.humOk = humidity_rel_ok;
  entry.presOk = pressure_ok;

  entry.tempInSamples = temperature_in_ok ? 1 : 0;
  entry.tempOutSamples = temperature_out_ok ? 1 : 0;
  entry.tempPcbSamples = temperature_pcb_ok ? 1 : 0;
  entry.humSamples = humidity_rel_ok ? 1 : 0;
  entry.presSamples = pressure_ok ? 1 : 0;

  logBufferHead = (logBufferHead + 1) % logBufferCapacity;
  if (logBufferCount < logBufferCapacity) {
    logBufferCount++;
  }

  Serial.println("RAM log write: epoch=" + String(entry.epoch)
                 + " used=" + String(logBufferCount)
                 + "/" + String(logBufferCapacity)
                 + " interval=" + String(effectiveLogIntervSec) + "s");

  newLogData = true;
}
#endif

void storeMotorPos() {
  motorPosPrefs.begin("motorpos", false);
  unsigned int currentTop = motor1->getTargetPosition();
  unsigned int currentBottom = motor2->getTargetPosition();

  bool hasTop = motorPosPrefs.isKey("top");
  bool hasBottom = motorPosPrefs.isKey("bottom");

  if (!hasTop || motorPosPrefs.getUInt("top", 0) != currentTop) {
    motorPosPrefs.putUInt("top", currentTop);
  }
  if (!hasBottom || motorPosPrefs.getUInt("bottom", 0) != currentBottom) {
    motorPosPrefs.putUInt("bottom", currentBottom);
  }
  motorPosPrefs.end();
}

void readMotorPos() {
  motorPosPrefs.begin("motorpos", true);  // read-only
  storedDialPosTop = motorPosPrefs.getUInt("top", 0);
  storedDialPosBottom = motorPosPrefs.getUInt("bottom", 0);
  motorPosPrefs.end();
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int max(int a, int b) {
  return (a > b) ? a : b;
}

float max(float a, float b) {
  return (a > b) ? a : b;
}

int min(int a, int b) {
  return (a < b) ? a : b;
}

float min(float a, float b) {
  return (a < b) ? a : b;
}

void displayScreensaver() {
  // Calculate text width
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  bool indoorValid = temperature_in_ok && temperature_in != INVALID_NUMBER;
  bool outdoorValid = temperature_out_ok && temperature_out != INVALID_NUMBER;

  if (indoorValid && outdoorValid) {
    if (screensaverShowOutdoor) {
      screensaverText = "out " + String(temperature_out, 1) + " C";
    } else {
      screensaverText = "in " + String(temperature_in, 1) + " C";
    }
    screensaverShowOutdoor = !screensaverShowOutdoor;
  } else if (outdoorValid) {
    screensaverText = "out " + String(temperature_out, 1) + " C";
  } else if (indoorValid) {
    screensaverText = "in " + String(temperature_in, 1) + " C";
  } else {
    screensaverText = "No temp data";
  }

  marqueeTextWidth = screensaverText.length() * charWidth * 2; // Adjust for text size 2

  int minX = 0;
  int maxX = SCREEN_WIDTH - marqueeTextWidth;
  // If just started or finished, pick new direction and Y
  if (marqueeX <= minX || marqueeX >= maxX) {
    marqueeY = random(0, SCREEN_HEIGHT - charHeight * 2); // Adjust for text size 2
    marqueeDir = (random(0, 2) == 0) ? 1 : -1;
    // Clamp direction if at edge
    if (marqueeX <= minX) marqueeDir = 1;
    if (marqueeX >= maxX) marqueeDir = -1;
  }
  display.clearDisplay();
  display.setCursor(marqueeX, marqueeY);
  display.print(screensaverText);
  display.display();
  marqueeX += marqueeDir;
  // Clamp to bounds
  if (marqueeX < minX) marqueeX = minX;
  if (marqueeX > maxX) marqueeX = maxX;
}

void forecastWeather() {
  forecastPresGradDir = 0;
  forecastMinutes = 0;

  for (int i = PRESSURE_HISTORY_SIZE - 1; i >= 0; i--) {
    if (pressureHistory[i - 1] == INVALID_NUMBER || pressureHistory[i] == INVALID_NUMBER) {
      break;  // Stop if we hit an invalid number
    } else {
      if (pressureHistory[i] - pressureHistory[i - 1] > PRESSURE_HISTORY_DIFFERENCE) {
        if (forecastPresGradDir < 0) {
          break; // Stop if the trend changes
        } else {
          forecastPresGradDir = 1;
        }
      } else if (pressureHistory[i] - pressureHistory[i - 1] < -PRESSURE_HISTORY_DIFFERENCE) {
        if (forecastPresGradDir > 0) {
          break; // Stop if the trend changes
        } else {
          forecastPresGradDir = -1;
        }
      } else if (abs(pressureHistory[i] - pressureHistory[PRESSURE_HISTORY_SIZE - 1]) > PRESSURE_HISTORY_DIFFERENCE) {
        break;  // Break if pressure gradient is small but the last value is significantly different from first
      }
      forecastMinutes += PRESSURE_HISTORY_INTERVAL_MINUTES;
    }
  }
}

void displayForecast() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.println("");
  display.println("Weather Forecast");
  display.println("");

  unsigned int hours = forecastMinutes / 60;  // Convert to hours

  // Display the pressure gradient direction
  if (forecastMinutes > 0) {
    display.print("Last ");
    if (hours > 0) {
      display.print(hours);
      display.print(" hours:");
    } else {
      display.print(forecastMinutes);
      display.print(" minutes:");
    }
    display.print(" pressure ");
    if (forecastPresGradDir > 0) {
      display.println("rising");
    } else if (forecastPresGradDir < 0) {
      display.println("falling");
    } else {
      display.println("stable");
    }
  } else {
    display.println("No pressure data available");
  }

  //debug print the pressure history
  for (int i = PRESSURE_HISTORY_SIZE - 1; i >= 0; i--) {
    if (pressureHistory[i] != INVALID_NUMBER) {
      display.print(pressureHistory[i]);
      display.print(" ");
    }
  }

  display.display();
}

void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  static bool connecting = false;
  const unsigned long retryInterval = 5000; // 5 seconds

  if (mqttClient.connected()) {
    connecting = false;
    return;
  }

  unsigned long now = millis();
  if (!connecting || now - lastAttempt > retryInterval) {
    connecting = true;
    lastAttempt = now;
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("GoodlyWeatherStation")) {
      Serial.println("connected");
      connecting = false;
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

void publishSensorDataToMQTT() {
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    char payload[32];
    if (temperature_in_ok) {
      snprintf(payload, sizeof(payload), "%.2f", temperature_in);
      mqttClient.publish("home/gws/temperature_in", payload, false);
    }
    if (temperature_out_ok) {
      snprintf(payload, sizeof(payload), "%.2f", temperature_out);
      mqttClient.publish("home/gws/temperature_out", payload, false);
    }
    if (temperature_pcb_ok) {
      snprintf(payload, sizeof(payload), "%.2f", temperature_pcb);
      mqttClient.publish("home/gws/temperature_pcb", payload, false);
    }
    if (pressure_ok) {
      snprintf(payload, sizeof(payload), "%.2f", pressure);
      mqttClient.publish("home/gws/pressure", payload, false);
    }
    if (humidity_rel_ok) {
      snprintf(payload, sizeof(payload), "%.2f", humidity_rel);
      mqttClient.publish("home/gws/humidity", payload, false);
    }
  }
}