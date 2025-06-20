//----------------------------------------------------------------------
// https://github.com/clearwater/SwitecX25
//

//----------------------------------------------------------------------

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <OneWire.h>
//#include <time.h>
#include <DHT22.h>
#include <RTClib.h>
//#include <DS18B20.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "src/SwitecX25/SwitecX25.h"
#include "src/NTPClient/NTPClient.h"

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

#define PIN_ONEWIRE 5  // ESP32 GPIO pin due to OneWire.h implementation

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
#define DATA_LOG_FILE "/datalog.txt"
#define DATA_LOG_HEADER "Date,Time,TempIn,TempOut,TempPcb,HumidityIn,PressureIn"
#define DIAL_POS_FILE "/dialpos.txt"

#define WIFI_FILE "/wifi.txt"
#define WIFI_RETRY_SEC 10

#define TEMP_PLAUS_MIN -60
#define TEMP_PLAUS_MAX 60
#define PRES_PLAUS_MIN 800
#define PRES_PLAUS_MAX 1200
#define HUM_PLAUS_MIN 0
#define HUM_PLAUS_MAX 100

#define NTP_TIMEOUT_SEC 5

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
unsigned int currentMessageLine = 0;
String messages[DISPLAY_LINES];
unsigned int storedDialPosTop;
unsigned int storedDialPosBottom;
float plotHours[] = { 1.0, 12, 24, 24 * 7, 24 * 30, 24 * 365, 24 * 365 * 2, 24 * 365 * 5, 24 * 365 * 10, 24 * 365 * 20, 24 * 365 * 50, 24 * 365 * 100 };
unsigned int plotHoursIdx = 2;
bool newLogData;
bool updatePlot;
bool updateAxes;
bool newSensorReadings = false;
bool logging = false;
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
//DS18B20 ds(PIN_ONEWIRE);
//uint8_t ds_address[] = { 40, 250, 31, 218, 4, 0, 0, 52 };
//uint8_t ds_selected;
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds(&oneWire);
float ds_temperature;
bool ds_temperature_ok = false;

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
unsigned int marqueeSpeed = 50; // ms between marquee updates
String screensaverText = "Goodly Weather Station";
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

void setup() {
  initSerial();
  initDisplay();
  initEncoder();
  initBmp();
  initDs();
  initSd();
  //removeDataLogFile();
  initDataLogFile();
  //initWifi();
  //initTime();
  // initDht();
  initSteppers();
  readSensors();
  arbitrateSensorReadings();
  getTimeStamp();
  updateDialPos(motor1, dialStateTop);
  updateDialPos(motor2, dialStateBottom);

  encPosPrev = encoder_position;
  encPushdPrev = true;
  lastInputMillis = millis();
  lastLogMillis = millis();  //Delay first log (sensor startup?)
  messageActive = false;
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
                  plotHoursIdx = 2;
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
                } else if (isMenuSelection("Bottom dial")) {
                  if (dialStateBottom == DialState::OFF) {
                    dialStateBottom = (DialState)0;
                  } else {
                    dialStateBottom = (DialState)((int)dialStateBottom + 1);
                  }
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
            setDisplayState(DisplayState::MENU);
            menuState = MenuState::MAIN;
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

  if ((millis() - lastSensorReadMillis > SENSOR_READ_INTERV_SEC * 1000)
      && (millis() - lastInputMillis > SENSOR_READ_INPUT_DLY_SEC * 1000)) {
    readSensors();
    arbitrateSensorReadings();
    forecastWeather();
    getTimeStamp();
    if (timeOk && millis() - lastLogMillis > LOG_INTERV_SEC * 1000 && logging) {
      logData();
      lastLogMillis = millis();
      //catFileSerial(DATA_LOG_FILE);
    }
  }

  updateDialPos(motor1, dialStateTop);
  updateDialPos(motor2, dialStateBottom);

  motor1->update();
  motor2->update();
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

void initDs() {
  // DS18B20 INIT
  displayMessage("Initializing DS18B20");
  //ds_selected = ds.select(ds_address);
  ds.begin();
  ds.setWaitForConversion(true);
  ds.requestTemperatures();
}

void zeroDials() {
  //motor1->setPosition(DIAL_RANGE_STEPS / 2);
  //motor2->setPosition(DIAL_RANGE_STEPS / 2);

  //motor1->updateBlocking();
  //motor2->updateBlocking();

  //storeMotorPos();

  displayMessage("Plz center top dial");
  zeroStepper(motor1);

  displayMessage("Plz center bottom dial");
  zeroStepper(motor2);
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
  String password;
  String ssid;
  displayMessage("Initializing WiFi");
  File file = SD.open(WIFI_FILE);
  if (file) {
    if (file.available()) {
      ssid = file.readStringUntil('\n');
      ssid.trim();
      // ssid = line.c_str();
      displayMessage("WiFi: " + ssid);
    }
    if (file.available()) {
      password = file.readStringUntil('\n');
      password.trim();
      // password = line.c_str();
      //displayMessage(password);
    }
  } else {
    displayMessage("Could not open " + String(WIFI_FILE));
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
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
  if (ds_temperature > TEMP_PLAUS_MIN && ds_temperature < TEMP_PLAUS_MAX && ds_temperature_ok) {
    temperature_out_array[avgSamplesTempOut - 1] = ds_temperature;
  } else {
    temperature_out_array[avgSamplesTempOut - 1] = INVALID_NUMBER;
  }
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
  //storeMotorPos();
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
}

void readBmp() {
  sensors_event_t event;
  bmp.getEvent(&event);
  bmp_pressure = event.pressure;
  bmp.getTemperature(&bmp_temperature);
}

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
  readDs();
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

  display.print("Temp ds: ");
  display.print(ds_temperature);
  display.println(" C");

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

  display.print("Temp out: ");
  display.print(temperature_out);
  display.println(" C");

  display.print("Pressure: ");
  display.print(pressure);
  display.println(" hPa");

  display.print("Humidity: ");
  display.print(humidity_rel);
  display.println(" %");

  display.print("\n");

  display.println(dateStamp + " " + timeStamp);

  if (logging) {
    display.print("Logging: ON");
  } else {
    display.print("Logging: OFF");
  }

  display.display();
}

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

void storeMotorPos() {
  File file = SD.open(DIAL_POS_FILE, FILE_WRITE);
  if (file) {
    file.println(motor1->getTargetPosition());
    file.println(motor2->getTargetPosition());
    file.close();
    //catFileSerial(DIAL_POS_FILE);
  } else {
    displayMessage("Error writing " + String(DIAL_POS_FILE));
  }
}

void readMotorPos() {
  File file = SD.open(DIAL_POS_FILE, FILE_READ);
  if (file) {
    String lineOne = file.readStringUntil('\n');
    //displayMessage("Line one: " + lineOne);
    String lineTwo = file.readStringUntil('\n');
    lineOne.trim();
    lineTwo.trim();
    storedDialPosTop = (unsigned int)lineOne.toInt();
    storedDialPosBottom = (unsigned int)lineTwo.toInt();
  } else {
    displayMessage("Error reading " + String(DIAL_POS_FILE));
  }
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
  screensaverText = String(temperature_out, 1) + " C";
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