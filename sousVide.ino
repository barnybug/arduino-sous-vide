/*
Arduino powered sous vide controller

Requires:
PID library found at
http://www.arduino.cc/playground/Code/PIDLibrary

dallasTemperature library found at
http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library

Pin configuration:
- LCD pins: 8, 9, 4, 5, 6, 7
- Analog 0: multi-level for buttons
- Backlight control: 10

andy@chiefmarley.com
2/9/2011
Adapted to dfrobot lcd and added transmit of temperature - barnaby@pickle.me.uk
26/2/2013
*/

#include <TimerOne.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <LCDKeypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <thgr810.h>

// period in milliseconds
#define UPDATE_INTERVAL 2500
// clock divisor - microseconds - 25ms (OUTPUT_LIMIT = UPDATE_INTERVAL / POLL_INTERVAL)
#define POLL_INTERVAL 25000
#define OUTPUT_LIMIT 100

// PID Parameters
#define PID_P 70
#define PID_I 0
#define PID_D 300

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// initial set point
#define INITIAL_SET_POINT 60

// pin to trigger relay
#define TRIGGER_PIN 13

// transmitter pin
#define TRANSMIT_CHANNEL 1
#define TRANSMIT_PIN 12
#define TRANSMIT_CODE 0x2b

// pin for backlight control
#define BACKLIGHT_PIN 10

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// LCD & Keypad library for dfrobot shield
LCDKeypad lcd;

//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
PID pid(&input, &output, &setpoint, PID_P, PID_I, PID_D, DIRECT, UPDATE_INTERVAL);

Thgr810 sensor(TRANSMIT_CHANNEL, TRANSMIT_PIN, TRANSMIT_CODE);

// Backlight timeout (ms)
#define BACKLIGHT_TIMEOUT 60000
#define REPEAT_INITIAL 300
#define REPEAT_FAST    30
int repeat_delay = 0;

// Moving long 'PWM' window
unsigned long windowStartTime;
unsigned long timeout = 0;
unsigned long buttonRepeat = 0;

void setup()
{
  Timer1.initialize(POLL_INTERVAL);

  //initialize the variables we're linked to
  input = 10;
  setpoint = INITIAL_SET_POINT;
  output = 50;
  pid.SetOutputLimits(0, OUTPUT_LIMIT);
  // Start up the library
  sensors.begin();
  //turn the PID on
  pid.SetMode(AUTOMATIC);
  lcd.begin(16,2);

  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);
  timeout = millis() + BACKLIGHT_TIMEOUT;
  pinMode(TRIGGER_PIN, OUTPUT);
  
  windowStartTime = millis() - UPDATE_INTERVAL;

  Timer1.attachInterrupt(timerInterrupt);
}

void loop()
{  
  unsigned long now = millis();
  if (now - windowStartTime > UPDATE_INTERVAL)
  {
    // update readings
    sensors.requestTemperatures(); // Send the command to get temperatures  
    input = sensors.getTempCByIndex(0);
    pid.Compute();
    updateLCD();
    transmit();
    windowStartTime += UPDATE_INTERVAL;
  }

  if (now > buttonRepeat) {
    int b = lcd.button();
    switch(b) {
      case KEYPAD_UP:
        setpoint += 0.1;
        break;
      case KEYPAD_DOWN:
        setpoint -= 0.1;
        break;
    }

    switch(b) {
      case KEYPAD_NONE:
        // reset delay on button repeat
        repeat_delay = REPEAT_INITIAL;
        break;
      default:
        // if a button was pressed, illuminate backlight
        digitalWrite(BACKLIGHT_PIN, HIGH);
        // and reset the backlight timeout
        timeout = now + BACKLIGHT_TIMEOUT;
        // set the next time to check buttons
        buttonRepeat = now + repeat_delay;
        // and set the button repeat to fast
        repeat_delay = REPEAT_FAST;
        updateLCD();
        break;
    }
  }
  
  if (now > timeout)
    digitalWrite(BACKLIGHT_PIN, LOW);
}

int counter = 0;
void timerInterrupt()
{
  digitalWrite(TRIGGER_PIN, counter < output);
  if (++counter >= OUTPUT_LIMIT)
    counter = 0;
}

void transmit()
{
  // transmit the input temperature as sensor temperature
  // and the output duty as humidity (out of 100)
  sensor.transmit(input, output*100/OUTPUT_LIMIT);
}

void updateLCD()
{
  lcd.clear();
  lcd.print("Act: ");
  lcd.print(input);
  lcd.setCursor(0, 1);
  lcd.print("Set: ");
  lcd.print(setpoint);
  lcd.setCursor(12, 1);
  lcd.print(output);
}  
