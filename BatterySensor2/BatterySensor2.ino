/*
 *  Battery-powered MySensors-2.0 sensor
 *
 *  requires: OneWire, DallasTemperature
 */

#include <avr/sleep.h>  // Sleep Modes
#include <OneWire.h>
#include <DallasTemperature.h>

// Dallas Settings
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
#define NUM_DIGITS 3 // Number of digits in float


// MySensors settings
#define MY_DEBUG
#define MY_RADIO_NRF24
// #define MY_BAUD_RATE 9600
#define MY_LEDS_BLINKING_FEATURE
#define MY_DEFAULT_LED_BLINK_PERIOD 300

#include <MySensors.h>
#include <SPI.h>

#define DEBUG

#define SKETCH_NAME "Multiple Dallas Sensors"
#define SKETCH_MAJOR_VER "0"
#define SKETCH_MINOR_VER "2"
#define BATTERY_SENSOR 0

unsigned long SLEEP_TIME = 10 * 60 * 1000L; // h*min*sec*1000ms
int unusedPins[] = {3, 4, 5, 6, 7, 8};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Globals
int oldBatLevel; // Old battery level
int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // Found device address

// MySensors messages
MyMessage vMsg(BATTERY_SENSOR, V_VOLTAGE);
MyMessage msg(1, V_TEMP); // Sensor Id will be dynamic

/*
 * MySensors 2,0 presentation
 */
void presentation() {
#ifdef DEBUG
  Serial.println("presentation");
#endif
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);
  present(BATTERY_SENSOR, S_MULTIMETER, "Battery Voltage");
  for (int i = 0; i < numberOfDevices; i++) {
    present(i + 1, S_TEMP); // start from 1
  }
}

/*
 * Setup
 */
void setup()
{
#ifdef DEBUG
  Serial.println("setup");
#endif
  // Reset pins
  int count = sizeof(unusedPins)/sizeof(int);
  for (int i = 0; i < count; i++) {
    pinMode(unusedPins[i], INPUT);
    digitalWrite(unusedPins[i], LOW);
  }
  oldBatLevel = -1;

  // Set up sensors
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Set up each device
  for (int i = 0; i < numberOfDevices; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Device ");
      Serial.print(i, DEC);
      Serial.print(" address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
      // Setting resolution
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  } // for
}

/*
 * Loop
 */
void loop() {
#ifdef DEBUG
  Serial.println("loop");
#endif
  if (oldBatLevel == -1) { // first start
    // Send the values before sleeping
    sendValues();
  }
  // Go to sleep
  sleep(SLEEP_TIME);
  // Read sensors and send on wakeup
  sendValues();
}

/*
 * Send sensor and battery values
 */
void sendValues()
{
#ifdef DEBUG
  Serial.println("sendValues");
#endif
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("done.");

  // Send sensor values
  for (int i = 0; i < numberOfDevices; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      float temp = sensors.getTempC(tempDeviceAddress);
#ifdef DEBUG
      Serial.print("Temperature for device ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(temp);
#endif
      msg.setSensor(i + 1); // start from 1
      send(msg.set(temp, NUM_DIGITS), true);
    } else {
      Serial.print("Found ghost device while reading temp at ");
      Serial.println(i, DEC);
    }
  } // for
  // Send battery level
  long vcc = readVcc();
  float v = vcc / 1000.0;
  send(vMsg.set(v, 3));
  // get percentage
  int batLevel = getBatteryLevel(vcc);
  if (oldBatLevel != batLevel) {
    sendBatteryLevel(batLevel);
    oldBatLevel = batLevel;
  }
}

/*
 * Battery measure
 */
int getBatteryLevel(long vcc) {
  if (vcc == NULL)
    vcc = readVcc();
  int results = (vcc - 2000)  / 10;
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

/*
 * Print a device address
 */
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
