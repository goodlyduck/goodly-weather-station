//----------------------------------------------------------------------
// https://github.com/clearwater/SwitecX25
//

//----------------------------------------------------------------------

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <DHT22.h>
#include <DS18B20.h>
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
  OFF,
  SENSORS,
  MENU,
  MENU_SET_TIME,
  MENU_ZERO_MOTOR_1,
  MENU_ZERO_MOTOR_2,
  MESSAGES
};

DisplayState displayState = DisplayState::INIT;
DisplayState displayStatePrev = DisplayState::INIT;

enum class DialState {
  INIT,
  TEMPERATURE_IN,
  TEMPERATURE_OUT,
  PRESSURE,
  HUMIDITY
};

DialState dialStateTop = DialState::TEMPERATURE_IN;
DialState dialStateBottom = DialState::HUMIDITY;

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

#define PIN_DS18B20 5  // ESP32 GPIO pin due to OneWire.h implementation

// USER DEFINES
#define DISPLAY_OFF_SEC 60
#define DISPLAY_LINES 8   // For font size 1
#define DISPLAY_CHARS 21  // For font size 1
#define SENSOR_READ_INTERV_SEC 60
#define SENSOR_READ_INPUT_DLY_SEC 5
#define ENCODER_DEBOUNCE_MILLIS 5
#define DATA_LOG_FILE "/datalog.txt"
#define DATA_LOG_HEADER "Date,Time,TempIn,TempOut,TempPcb,HumidityIn,PressureIn"
#define DIAL_POS_FILE "/dialpos.txt"

// ROTARY ENCODER PUSH BUTTON
// detachInterrupt(GPIOPin);
volatile bool encoder_clock_prev;
volatile bool encoder_cw_pressed = false;
volatile bool encoder_ccw_pressed = false;
volatile bool encoder_push_pressed = false;
volatile int encoder_position = 0;

// USER CONSTANTS
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;

// USER VARIABLES
volatile unsigned long lastInputMillis = millis();
unsigned long lastSensorReadMillis = 0;
float temperature_in;
float temperature_out;
float temperature_pcb;
float pressure;
float humidity_rel;
unsigned int currentMessageLine = 0;
String messages[DISPLAY_LINES];
unsigned int storedDialPosTop;
unsigned int storedDialPosBottom;

// DHT22 TEMP AND HUMIDITY SENSOR
DHT22 dht22(PIN_DHT);
float dht_temperature;
float dht_humidity_rel;

// BMP180 PRESSURE (AND TEMP) SENSOR (DEFAULT I2C PINS A4, A5)
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
float bmp_temperature;
float bmp_pressure;

// DS18B20 1wire temp sensor
DS18B20 ds(PIN_DS18B20);
uint8_t ds_address[] = { 40, 250, 31, 218, 4, 0, 0, 52 };
uint8_t ds_selected;
float ds_temperature;

// STEPPER MOTOR DEFINES
// standard X25.168 range 315 degrees at 1/3 degree steps
// #define STEPS (315*3)
// #define STEPS (360*3)
// #define DIAL_RANGE_DEG 216 // Limited to front markings
#define DIAL_RANGE_DEG 252    // Limited to front markings plus one more
#define DIAL_RANGE_STEPS (DIAL_RANGE_DEG * 3)
#define DIAL_CAL_STEPS 2
// SwitecX25 motor1(STEPS,9,7,5,3);
SwitecX25 *motor1;
SwitecX25 *motor2;

// OLED DISPLAY DEFINES
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         &SPI, PIN_SPI_DC, PIN_OLED_RESET, PIN_SPI_CS_OLED);
bool displayOn = true;

void setup() {
  initSerial();
  initDisplay();
  initEncoder();
  initBmp();
  initDs();
  initSd();
  initWifi();
  initTime();
  // initDht();
  initSteppers();

  readSensors();
  arbitrateSensorReadings();
  updateDialPos(motor1, dialStateTop);
  updateDialPos(motor2, dialStateBottom);
}

void loop() {
  if (millis() - lastInputMillis > DISPLAY_OFF_SEC * 1000) {
    setDisplayOff();
    displayOn = false;
  } else {
    displayOn = true;
  }

  if (millis() - lastSensorReadMillis > SENSOR_READ_INTERV_SEC * 1000) {
    readSensors();
    arbitrateSensorReadings();
    if (timeClient.isTimeSet()) {
      logData();
      catFileSerial(DATA_LOG_FILE);
    }
    updateDialPos(motor1, dialStateTop);
    updateDialPos(motor2, dialStateBottom);
  }

  if (displayOn) {
    printSensorReadings();
  }
  motor1->update();
  motor2->update();

  // if (timeClient.isTimeSet())
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
    SD.remove(DATA_LOG_FILE);  // Recreate datalog file each time
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
    if (!SD.exists(DATA_LOG_FILE)) {
      File dataFile = SD.open(DATA_LOG_FILE, FILE_WRITE);
      if (dataFile) {
        dataFile.print(DATA_LOG_HEADER);
        dataFile.print("\r\n");
        dataFile.close();
        displayMessage(String(DATA_LOG_FILE) + " created");
        catFileSerial(DATA_LOG_FILE);
      } else {
        displayMessage("Failed to create + " DATA_LOG_FILE);
      }
    }
  }
}

void initEncoder() {
  // ROTARY ENCODER INTERRUPT
  pinMode(PIN_ENCODER_CLK, INPUT);
  pinMode(PIN_ENCODER_DT, INPUT);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_CLK, isrEncoderClock, CHANGE);
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
  ds_selected = ds.select(ds_address);
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
  displayMessage("Top: " + String(storedDialPosTop));
  motor2->setCurrentStep(storedDialPosBottom);
  displayMessage("Bottom: " + String(storedDialPosBottom));

  motor1->setPosition(DIAL_RANGE_STEPS/2);
  motor2->setPosition(DIAL_RANGE_STEPS/2);

  motor1->updateBlocking();
  motor2->updateBlocking();

  displayMessage("Plz center top dial");
  zeroStepper(motor1);

  displayMessage("Plz center bottom dial");
  zeroStepper(motor2);
}

// INTERRUPT FUNCTIONS

void isrEncoderClock() {
  // if (millis() - lastInputMillis < ENCODER_DEBOUNCE_MILLIS)
  bool encoder_clock = digitalRead(PIN_ENCODER_CLK);
  if (encoder_clock != encoder_clock_prev) {
    bool encoder_dt = digitalRead(PIN_ENCODER_DT);
    if (encoder_dt != encoder_clock) {
      // encoder_cw_pressed = true;
      encoder_position++;
    } else {
      // encoder_ccw_pressed = true;
      encoder_position--;
    }
    encoder_clock_prev = encoder_clock;
  }
  lastInputMillis = millis();
}

// IRAM_ATTR?
void isrEncoderPush() {
  encoder_push_pressed = true;
  lastInputMillis = millis();
}

// OTHER FUNCTIONS

void arbitrateSensorReadings() {
  temperature_in = dht_temperature;
  temperature_out = ds_temperature;
  temperature_pcb = bmp_temperature;
  pressure = bmp_pressure;
  humidity_rel = dht_humidity_rel;
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
      break;
    case DialState::PRESSURE:
      position = valToDialPos(pressure, DIAL_PRESSURE_MIN, DIAL_PRESSURE_MAX);
      motor->setPosition(position);
      break;
    case DialState::HUMIDITY:
      position = valToDialPos(humidity_rel, DIAL_HUMIDITY_MIN, DIAL_HUMIDITY_MAX);
      motor->setPosition(position);
      break;
  }
  storeMotorPos();
}

unsigned int valToDialPos(float val, float min, float max) {
  return map(val, min, max, 0, DIAL_RANGE_STEPS);
}

void displayMessage(const String &message) {

  // int newLines = message.length()/DISPLAY_CHARS;

  // Change display state?
  if (displayState != DisplayState::MESSAGES) {
    displayState = DisplayState::MESSAGES;
  }

  // If buffer is full, shift all lines up
  if (currentMessageLine >= DISPLAY_LINES) {
    for (int i = 1; i < DISPLAY_LINES; i++) {
      messages[i - 1] = messages[i];
    }
    currentMessageLine--;
  }

  // Add new line to buffer
  messages[currentMessageLine] = message;
  currentMessageLine++;

  // Redraw display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (int i = 0; i < currentMessageLine; i++) {
    display.setCursor(0, i * 8);  // Each line is 8 pixels tall
    display.println(messages[i]);
  }

  display.display();

  Serial.println(message);

  delay(200);
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
  display.clearDisplay();
  display.display();
}

void readBmp() {
  sensors_event_t event;
  bmp.getEvent(&event);
  bmp_pressure = event.pressure;
  bmp.getTemperature(&bmp_temperature);
}

void readDs() {
  ds_temperature = ds.getTempC();
}

void readDht() {
  dht_temperature = dht22.getTemperature();
  // delay(50);  // For stable readings
  dht_humidity_rel = dht22.getHumidity();
}

void readSensors() {
  readDht();
  readBmp();
  readDs();
  lastSensorReadMillis = millis();
}

void printSensorReadings() {
  display.clearDisplay();
  display.setCursor(0, 0);  // Start at top-left corner

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

  display.print("\n");

  display.print("Encoder: ");
  display.print(encoder_position);

  display.display();
}

void initWifi() {
  String password;
  String ssid;
  displayMessage("Initializing WiFi");
  File file = SD.open("/wifi.txt");
  if (file) {
    if (file.available()) {
      ssid = file.readStringUntil('\n');
      ssid.trim();
      // ssid = line.c_str();
      displayMessage(ssid);
    }
    if (file.available()) {
      password = file.readStringUntil('\n');
      password.trim();
      // password = line.c_str();
      displayMessage(password);
    }
  } else {
    displayMessage("Could not open /wifi.txt");
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

void initTime() {
  displayMessage("Initializing NTP time");
  timeClient.begin();
  getTimeStamp();
  displayMessage(dateStamp + " " + timeStamp);
}


// Function to get date and time from NTPClient
void getTimeStamp() {
  while (!timeClient.update()) {
    timeClient.forceUpdate();
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
}

void logData() {
  // "Date,Time,TempIn,TempOut,TempPcb,HumidityIn,PressureIn"
  File file = SD.open(DATA_LOG_FILE, FILE_WRITE);
  if (file) {
    file.seek(file.size());
    file.print(dateStamp);
    file.print(",");
    file.print(timeStamp);
    file.print(",");
    file.print(temperature_in, 1);
    file.print(",");
    file.print(temperature_out, 1);
    file.print(",");
    file.print(temperature_pcb, 1);
    file.print(",");
    file.print(humidity_rel, 1);
    file.print(",");
    file.print(pressure, 1);
    file.print("\r\n");
    file.close();
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
    catFileSerial(DIAL_POS_FILE);
  } else {
    displayMessage("Error writing " + String(DIAL_POS_FILE));
  }
}

void readMotorPos() {
  File file = SD.open(DIAL_POS_FILE, FILE_READ);
  if (file) {
    String lineOne = file.readStringUntil('\n');
    displayMessage("Line one: " + lineOne);
    String lineTwo = file.readStringUntil('\n');
    lineOne.trim();
    lineTwo.trim();
    storedDialPosTop = (unsigned int)lineOne.toInt();
    storedDialPosBottom = (unsigned int)lineTwo.toInt();
  } else {
    displayMessage("Error reading " + String(DIAL_POS_FILE));
  }
}

// void updateDisplay() {
//   switch(displayState) {
//     case MENU_ZERO_MOTOR_1:

//   }
//   //   INIT,
//   // OFF,
//   // SENSORS,
//   // MENU,
//   // MENU_SET_TIME,
//   // MENU_ZERO_MOTOR_1,
//   // MENU_ZERO_MOTOR_2,
//   // MESSAGES
// }