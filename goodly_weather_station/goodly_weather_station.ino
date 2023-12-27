//----------------------------------------------------------------------
// https://github.com/clearwater/SwitecX25
// 

//----------------------------------------------------------------------

//#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT11.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "src/SwitecX25/SwitecX25.h"


// STATES
enum displayState {
  OFF,
  SENSORS,
  MENU
};

// PINS
//#define PIN_SPI_MOSI   3
//#define PIN_SPI_MISO   9
//#define PIN_SPI_CLK   2
#define PIN_SPI_DC 9
#define PIN_SPI_CS_OLED 10
#define PIN_SPI_CS_SD 7
#define PIN_OLED_RESET 8
#define PIN_DHT 6
#define MOTOR1_PIN1 A0
#define MOTOR1_PIN2 A1
#define MOTOR1_PIN3 A2
#define MOTOR1_PIN4 A3

#define PIN_ENCODER_CLK 5
#define PIN_ENCODER_DT 4
#define PIN_ENCODER_SW 3 


// USER DEFINES
#define DISPLAY_OFF_SEC 20
#define SENSOR_READ_INTERV_SEC 5
#define ENCODER_DEBOUNCE_MILLIS 5

// ROTARY ENCODER PUSH BUTTON
//detachInterrupt(GPIOPin);
volatile bool encoder_clock_prev;
volatile bool encoder_cw_pressed = false;
volatile bool encoder_ccw_pressed = false;
volatile bool encoder_push_pressed = false;
volatile int encoder_position = 0;

// USER VARIABLES
volatile unsigned long lastInputMillis = millis();
unsigned long lastSensorReadMillis = 0;

// DHT11 TEMP AND HUMIDITY SENSOR
DHT11 dht11(PIN_DHT);
int dht_temperature;
int dht_humidity_rel;


// BMP180 PRESSURE (AND TEMP) SENSOR (DEFAULT I2C PINS A4, A5)
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
float bmp_temperature;
float bmp_pressure;


// STEPPER MOTOR DEFINES
// standard X25.168 range 315 degrees at 1/3 degree steps
#define STEPS (315*3)
//SwitecX25 motor1(STEPS,9,7,5,3);
SwitecX25 *motor1;


//OLED DISPLAY DEFINES
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, PIN_SPI_DC, PIN_OLED_RESET, PIN_SPI_CS_OLED);

/*
// OLED display software SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  PIN_SPI_MOSI, PIN_SPI_CLK, PIN_SPI_DC, PIN_OLED_RESET, PIN_SPI_CS_OLED);
*/
bool displayOn = true;

void setup()
{
  Serial.begin(9600);
  while (!Serial) {}

  //SD CARD READER
  

  // ROTARY ENCODER INTERRUPT
  pinMode(PIN_ENCODER_CLK, INPUT);
  pinMode(PIN_ENCODER_DT, INPUT);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_CLK, isrEncoderClock, CHANGE);
  attachInterrupt(PIN_ENCODER_SW, isrEncoderPush, RISING);
  //encoder_clock_prev = (bool)digitalRead(PIN_ENCODER_CLK);

  // BMP180 INIT
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  // STEPPER MOTOR INIT
  // Setting pin mode in SwitecX25 constructor before setup() does not work
  // Create stepper object here
  motor1 = new SwitecX25(STEPS, MOTOR1_PIN1, MOTOR1_PIN1, MOTOR1_PIN1, MOTOR1_PIN1);
  // run the motor against the stops
  motor1->zero();
  // start moving towards the center of the range
  motor1->setPosition(STEPS/2);
  Serial.print("Enter a step position from 0 through ");
  Serial.print(STEPS-1);
  Serial.println(".");


  // OLED DISPLAY INIT
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(1000); 
  //testdrawcircle();
  // Clear the buffer
  display.clearDisplay();
  display.display();
  // Init text, enough to keep here?
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
}


void loop()
{
  if (millis() - lastInputMillis > DISPLAY_OFF_SEC*1000) {
    setDisplayOff();
    displayOn = false;
  }
  else {
    displayOn = true;
  }
  
  //displayEncoderReadings();

  if (encoder_push_pressed) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Encoder push");
    display.display();
    encoder_push_pressed = false;
    delay(1000);
  }

  if (encoder_cw_pressed) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Encoder clockwise");
    display.display();
    delay(1000);
    encoder_cw_pressed = false;
  }

  if (encoder_ccw_pressed) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Encoder counter clockwise");
    display.display();
    delay(1000);
    encoder_ccw_pressed = false;
  }

  if (millis() - lastSensorReadMillis > SENSOR_READ_INTERV_SEC*1000) {
    readSensors();
  }

  if (displayOn) {
    printSensorReadings();
  }

  /*
  static int nextPos = 0;
  // the motor only moves when you call update
  motor1->update();
  
  if (Serial.available()) {
    char c = Serial.read();
    if (c==10 || c==13) {
      motor1->setPosition(nextPos);
      nextPos = 0;
    } else if (c>='0' && c<='9') {
      nextPos = 10*nextPos + (c-'0');
    }
  }
  */
}


void testdrawcircle() {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}


void readBmp() {
  sensors_event_t event;
  bmp.getEvent(&event);
  bmp_pressure = event.pressure;
  bmp.getTemperature(&bmp_temperature);
}


void readDht() {
  dht_temperature = dht11.readTemperature();
  //delay(50);  // For stable readings
  dht_humidity_rel = dht11.readHumidity();
}


void readSensors() {
  readDht();
  readBmp();
  lastSensorReadMillis = millis();
}


void printSensorReadings() {
  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner

  display.print("Temp bmp: ");
  display.print(bmp_temperature);
  display.println(" C");

  display.print("Pressure: ");
  display.print(bmp_pressure);
  display.println(" hPa");

  display.print("\n");

  display.print("Temp dht: ");
  display.print(dht_temperature);
  display.println(" C");

  display.print("Humidity: ");
  display.print(dht_humidity_rel);
  display.println(" %");

  display.print("\n");

  display.print("Encoder: ");
  display.print(encoder_position);

  display.display();
}


void isrEncoderClock() {
  //if (millis() - lastInputMillis < ENCODER_DEBOUNCE_MILLIS)
  bool encoder_clock = digitalRead(PIN_ENCODER_CLK);
  if (encoder_clock != encoder_clock_prev) {
    bool encoder_dt = digitalRead(PIN_ENCODER_DT);
    if (encoder_dt != encoder_clock) {
      //encoder_cw_pressed = true;
      encoder_position++;
    }
    else {
      //encoder_ccw_pressed = true;
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

void initCardReader() {
  Serial.println("Initializing SD card...");
  Serial.println();
}