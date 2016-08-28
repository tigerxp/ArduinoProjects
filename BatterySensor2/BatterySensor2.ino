#include <Arduino.h>

/* 
 *  Battery-powered MySensors-2.0 sensor
 *  Multiple DS18B20 handling
 *  
 *  requires: OneWire, DallasTemperature
 */
 
#include <avr/sleep.h>  // Sleep Modes
#include <OneWire.h>
#include <DallasTemperature.h>

// Dallas Settings
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9 // Lower resolution

// MySensors settings
#define MY_DEBUG
#define MY_RADIO_NRF24
// #define MY_BAUD_RATE 9600
#define MY_LEDS_BLINKING_FEATURE
#define MY_DEFAULT_LED_BLINK_PERIOD 300

#include <MySensors.h>
#include <SPI.h>

#define DEBUG true

#define SKETCH_NAME "Battery Sensor"
#define SKETCH_MAJOR_VER "0"
#define SKETCH_MINOR_VER "1"

// unsigned long SLEEP_TIME = 24*60*60*1000; // h*min*sec*1000
unsigned long SLEEP_TIME = 60*1000; // 60s

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Globals
int oldBatLevel; // Old battery level

int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // Found device address

/*
 * MySensors 2,0 presentation
 */
void presentation() {
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);
}

/*
 * Sertup
 */
void setup()
{
#ifdef DEBUG
  Serial.println("Setup function");
#endif
  // Reset pins
  int pins[] = {2, 3, 4, 5, 6, 7, 8};
  int count = sizeof(pins)/sizeof(int);
  for (int i = 0; i < count; i++) {
    pinMode(pins[i], INPUT);
    digitalWrite(pins[i], LOW);
  }
  oldBatLevel = -1;

  // Set up sensors
  sensors.begin();
numberOfDevices = sensors.getDeviceCount();

  
}

/*
 * Loop
 */
void loop() {
#ifdef DEBUG
  Serial.println("Loop function");
#endif
  // Read sensors and send on wakeup
  
  sendValues();

  // Go to sleep
  sleep(SLEEP_TIME);
}

/*
 * Send sensor and battery values
 */
void sendValues()
{
#ifdef DEBUG
  Serial.println("sendValues function");
#endif

  // Send sensor values
  // ...
  
  // Send battery level
  int batLevel = getBatteryLevel();
  if (oldBatLevel != batLevel) {
    sendBatteryLevel(batLevel);
    oldBatLevel = batLevel;
  }
}

/*
 * Battery measure
 */
int getBatteryLevel() {
  int results = (readVcc() - 2000)  / 10;
  if (results > 100)
    results = 100;
  if (results < 0)
    results = 0;
  return results;
}

// when ADC completed, take an interrupt
EMPTY_INTERRUPT (ADC_vect);

/*
 * Tricky function to read value of the VCC
 */
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  noInterrupts();
  // start the conversion
  ADCSRA |= _BV(ADSC) | _BV(ADIE);
  set_sleep_mode(SLEEP_MODE_ADC); // sleep during sample
  interrupts();
  sleep_mode();
  // reading should be done, but better make sure
  // maybe the timer interrupt fired
  while(bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
#ifdef DEBUG
  Serial.print("Battery voltage is: ");
  Serial.print(result);
  Serial.println(" mV");
#endif
  return result;
}

