/*
 * PLoggerProtoRevA.ino
 * 
 * Copyright 2017 Zoltan DeWitt
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * Requires:
 *      ESP8266 Arduino: https://github.com/esp8266/Arduino
 *      ESP8266 WiFiManager: https://github.com/tzapu/WiFiManager
 *      ESP8266 TwoColorLed: https://github.com/zdewitt/TwoColorLed
 * 
 * This is a prototype sketch for a Power Logger device
 * It will connect over WiFi to the thinger.io platform and log power usage
 * of a single load.
 */

#define _DEBUG_  // for thinger.io debug messages

#include <FS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include "TwoColorLed.h"
#include "SPI.h"
#include <ThingerWifi.h>
#include <EEPROM.h>

/* define the thinger.io interface
 * the thingerCredentials.h file must contain the user/device info to connect
 * to the thinger web api:
 *    USERNAME - account login username
 *    DEVICE_ID - name of the device on the thinger platform
 *    DEVICE_CREDENTIAL - secret key to connect
 */
#include "thingerCredentials.h"
ThingerWifi thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

/* device status feedback LED
 * currently only responds to status of internet connection and valid calibration data
 */
TwoColorLed led( 4, 5, false );

// enable serial messages for hardware testing
#define DEBUG_MODE

/* perform a calibration at startup
 * this should only be performed once on the device by starting up with no load connected
 * if ADC_OVERSAMPLING_BITS is changed, calibration must be done again
 */
//#define RESET_CALIBRATION  // MUST comment this out after resetting once


// hardware ESP8266 pin definitions
#define CURRENT_PULSE_PIN 16
#define VOLTAGE_PULSE_PIN 13
#define CURRENT_LEVEL_CS_PIN 2
#define VOLTAGE_LEVEL_PIN A0


/* ADC factors for current/voltage conversions
 * the voltage factor (V/bit) is determined by the voltage divider (see schematic) and the ESP8266
 *    analog pin range, 0-1.0V
 * the current factor (A/bit) is determined by the ACS722 range (-20A,20A) and the bit-depth
 *    of the ADC device.
 */
#define ADC_VOLTAGE_FACTOR_10BIT 0.3873416081  // for analogReads, with 488.0/1.27 divider
#define ADC_CURRENT_FACTOR_12BIT .012207031 // for ADC121S021 and ACS722 at 3.3v

// oversampling the current sensor provides better value resolution, at the expense of temporal resolution
#define ADC_OVERSAMPLING_BITS 2  // the number of oversampling bits. maximum 4

const float_t ADC_VOLTAGE_FACTOR = ADC_VOLTAGE_FACTOR_10BIT;
const float_t ADC_CURRENT_FACTOR = ADC_CURRENT_FACTOR_12BIT / (1 << ADC_OVERSAMPLING_BITS);

const float_t SQRT2 = 1.4142136;
uint32_t g_adc_value_buffer = 0; // global buffers for higher-speed transfers
uint16_t g_adc_read_buffer = 0;


// SPI settings for the ADC121S021
const SPISettings ADC121S021_spi_settings( SPI_CLOCK_DIV8, MSBFIRST, SPI_MODE0 );

volatile uint32_t g_interruptTimerStart = 0;
volatile uint32_t g_interruptTimerEnd = 0;

float_t g_lineHz = 0;
float_t g_voltage = 0;
float_t g_voltagePk = 0;
float_t g_current = 0;
float_t g_powerReal = 0;
float_t g_powerFactor = 0;

uint16_t g_currentZeroCalibration = 0;

uint32_t g_lastUpdateTime = 0;

/* handles interrupts from a pin change and records up to two timestamps
 */
void ICACHE_RAM_ATTR timingHandler() {
  if (!g_interruptTimerStart) {
    g_interruptTimerStart = micros();
  } else if (!g_interruptTimerEnd) {
    g_interruptTimerEnd = micros();
  }
}

void setup() {
    SPI.begin();
    pinMode( CURRENT_LEVEL_CS_PIN, OUTPUT );
    digitalWrite( CURRENT_LEVEL_CS_PIN, HIGH );
    
    pinMode( CURRENT_PULSE_PIN, INPUT_PULLUP );
    pinMode( VOLTAGE_PULSE_PIN, INPUT_PULLUP );
    
    led.setColor( 0 );
    led.setLevel( 1023 );
    led.setBlink( 250 );
    
    Serial.begin(115200);

    //WiFiManager
    WiFiManager wifiManager;
    //reset saved settings. run this once to clear the access point info from EEPROM
    //wifiManager.resetSettings();
    
    // blink orange while connecting
    led.setColor( 700 );
    
    wifiManager.autoConnect();

    // solid green when connected
    led.setColor( 1023 );
    led.setBlink( 0 );
    
    //adcTestSPS();  // uncomment this to test the sampling rate at startup
    
    // Load previous calibration or re-calibrate if flag is set
    EEPROM.begin(4);
    #ifdef RESET_CALIBRATION
    calibrateCurrentSensor();
    #endif
    uint8_t highByte = EEPROM.read(0); // MSB first
    uint8_t lowByte = EEPROM.read(1);
    uint8_t checksum = EEPROM.read(2);
    #ifdef DEBUG_MODE
    Serial.print("Calibration read from EEPROM: ");
    Serial.print(highByte, HEX);
    Serial.print(" ");
    Serial.print(lowByte, HEX);
    Serial.print(" ");
    Serial.println(checksum, HEX);
    #endif
    if (checksum != (uint8_t)(highByte+lowByte)) {
      // calibration data is not valid, hang here
      led.setColor(0);
      led.setBlink(1000);
      while (1) {
        yield();
      }
    } else {
      g_currentZeroCalibration = highByte << 8;
      g_currentZeroCalibration += lowByte;
    }
    
    // Set up thinger.io interface
    thing["line_freq"] >> outputValue(g_lineHz);
    thing["volts_peak"] >> outputValue(g_voltage);
    thing["current_peak"] >> outputValue(g_current);
    thing["real_power"] >> outputValue(g_powerReal);
    thing["power_factor"] >> outputValue(g_powerFactor);
    
}

void loop() {
    if (millis() - g_lastUpdateTime >= 1000) {
      updateSenseValues();
      while (!cycleCalculation()) continue;
      g_lastUpdateTime = millis();
    }
    
    thing.handle();
}


/* sets the no-load value from the current sensor.
 * takes the average of 100 samples and writes to EEPROM
 */
void calibrateCurrentSensor() {
  uint32_t accumulator = 0;
  for (int i=0; i<100; i++) {
    accumulator += adcRead( CURRENT_LEVEL_CS_PIN );
    yield();
  }
  accumulator = accumulator/100;
  g_currentZeroCalibration = accumulator;
  uint8_t lowByte = g_currentZeroCalibration;
  uint8_t highByte = g_currentZeroCalibration >> 8;
  uint8_t checksum = highByte + lowByte;
  EEPROM.write(0, highByte);
  EEPROM.write(1, lowByte);
  EEPROM.write(2, (highByte + lowByte));
  EEPROM.commit();
#ifdef DEBUG_MODE
  Serial.print("Current calibration midpoint set at ");
  Serial.println(g_currentZeroCalibration, HEX);
  Serial.print(highByte, HEX);
  Serial.print(" ");
  Serial.print(lowByte, HEX);
  Serial.print(" ");
  Serial.println(checksum, HEX);
#endif
}


/* updates the line frequency, peak voltage, and RMS voltage global vars
 */
void updateSenseValues() {
    float_t lineHz = senseLineHz();
    float_t voltagePk = sensePeakVoltage() * ADC_VOLTAGE_FACTOR;
    float_t voltageRMS = voltagePk/SQRT2;
#ifdef DEBUG_MODE
    Serial.print( "Line freq: " );
    Serial.println( lineHz );
    Serial.print( "Line RMS voltage: " );
    Serial.println( voltageRMS );
#endif
    if ( (lineHz < 70) && (lineHz > 40) ) g_lineHz = lineHz;
    if ( (voltageRMS < 400) && (voltageRMS > 80) ) g_voltage = voltageRMS; g_voltagePk = voltagePk;
}


/* update the g_powerReal and g_current vars with new values.
 * performs a numerical integration on a single cycle of the
 * voltage and current waveforms. voltage is approximated as a
 * sin wave using the peak voltage and cycle period
 */
bool cycleCalculation() {
  // get a rising edge from the voltage pulse to synchronize the timing of the voltage waveform
  g_interruptTimerStart = 0;
  g_interruptTimerEnd = 0;
  attachInterrupt( VOLTAGE_PULSE_PIN, timingHandler, RISING );
  while (!g_interruptTimerEnd) {
    yield();
  }
  detachInterrupt( VOLTAGE_PULSE_PIN );
  
  // local vars for entire cycle
  long peakCurrent = 0;
  uint32_t maxTimeStep = 0;
  long numSteps = 0;
  float_t accumulatorPowerPositive = 0;  // split these out for eventual reactive power analysys
  float_t accumulatorPowerNegative = 0;
  uint32_t accumulatorRMSCurrent = 0;
  uint32_t cycleLengthMicros = g_interruptTimerEnd - g_interruptTimerStart;
  float_t radiansPerMicro = 2.0 * PI / (float_t)cycleLengthMicros;
  uint32_t cycleStartMicros = micros();
  
  // local vars for each integration step
  long lastCurrentADC = 0;
  long currentCurrentADC = 0;
  uint32_t currentMicros = cycleStartMicros;
  uint32_t lastMicros = currentMicros;
  long averageCurrentADC = 0;
  float_t averageRadians = 0;
  float_t averageNormalizedVoltage = 0;
  uint32_t timestepMicros = 0;
  float_t currentWeightedNormalizedPower = 0;
  
  /* calculate the integral for one cycle period.
   * we substitute sin(radians) for the voltage and multiply by the peak later.
   * the calculation is not synced with the cycle period, but the important factors
   * are that radians are correct and that the duration is one period.
   */
  lastCurrentADC = adcRead( CURRENT_LEVEL_CS_PIN ) - g_currentZeroCalibration;
  while ( lastMicros - cycleStartMicros <= cycleLengthMicros ) {
    ++numSteps;
    currentMicros = micros();
    currentCurrentADC = adcRead( CURRENT_LEVEL_CS_PIN ) - g_currentZeroCalibration;
    accumulatorRMSCurrent += (uint32_t)currentCurrentADC * (uint32_t)currentCurrentADC;
    if (abs(currentCurrentADC) > peakCurrent) {
      peakCurrent = abs(currentCurrentADC);
    }
    if (lastMicros) {
      // perform a numerical integration using straight line approximation
      averageCurrentADC = (lastCurrentADC + currentCurrentADC) / 2;
      averageRadians = (lastMicros - g_interruptTimerStart + currentMicros - g_interruptTimerStart) / 2 * radiansPerMicro;
      averageNormalizedVoltage = sin(averageRadians);
      timestepMicros = currentMicros - lastMicros;
      if (timestepMicros > maxTimeStep) maxTimeStep = timestepMicros;
      currentWeightedNormalizedPower = averageCurrentADC * averageNormalizedVoltage * timestepMicros;
      if (currentWeightedNormalizedPower > 0) {
        accumulatorPowerPositive += currentWeightedNormalizedPower;
      } else {
        accumulatorPowerNegative += currentWeightedNormalizedPower;
      }
    }
    lastCurrentADC = currentCurrentADC;
    lastMicros = currentMicros;
  }
  
  yield();
  
  // final conversions to real units and global var updates
  uint32_t totalMeasuredMicros = lastMicros - cycleStartMicros;
  float_t powerPositive = accumulatorPowerPositive * g_voltagePk * ADC_CURRENT_FACTOR / totalMeasuredMicros;
  float_t powerNegative = accumulatorPowerNegative * g_voltagePk * ADC_CURRENT_FACTOR / totalMeasuredMicros;
  float_t peakCurrentF = peakCurrent * ADC_CURRENT_FACTOR;
  float_t powerReal = (powerPositive + powerNegative);
  float_t powerFactor = powerReal / (g_voltage * sqrt(accumulatorRMSCurrent / numSteps) * ADC_CURRENT_FACTOR); // real power divided by RMS voltage * RMS current
  #ifdef DEBUG_MODE
  Serial.println();
  Serial.print("Measured power for this cycle: ");
  Serial.print(powerPositive, DEC);
  Serial.print(" ");
  Serial.println(powerNegative, DEC);
  Serial.print("Power factor: ");
  Serial.println(powerFactor, DEC);
  Serial.print("Peak current: ");
  Serial.println(peakCurrentF, DEC);
  Serial.print("Max timestep: ");
  Serial.println(maxTimeStep, DEC);
  Serial.print("Num steps: ");
  Serial.println(numSteps);
  Serial.print("Cycle start delay us: ");
  Serial.println(cycleStartMicros-g_interruptTimerStart);
  #endif
  
  if (maxTimeStep < 1000 && numSteps > 50) {
    g_powerReal = powerReal;
    g_powerFactor = powerFactor;
    g_current = peakCurrentF;
    return true;
  } else {
    #ifdef DEBUG_MODE
    Serial.println("Cycle failed");
    #endif
    return false;
  }
}


/* perform a read of an SPI ADC connected to the specified CS pin.
 * returns a raw value with a bit depth higher than the native depth
 * by the number of oversampling bits. 
 */
uint32_t adcRead( const uint8_t csPin ) {
  g_adc_value_buffer = 0;
  g_adc_read_buffer = 0;
  SPI.beginTransaction( ADC121S021_spi_settings );
  
  // explicit calls for oversampling < 4, for speed
#if ADC_OVERSAMPLING_BITS < 4
  adcReadRaw( csPin );
  #if ADC_OVERSAMPLING_BITS > 0
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  #endif
  #if ADC_OVERSAMPLING_BITS > 1
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  #endif
  #if ADC_OVERSAMPLING_BITS > 2
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  adcReadRaw( csPin );
  #endif
#else
  for (int i = 0; i<(4^ADC_OVERSAMPLING_BITS); i++) {
    adcReadRaw( csPin );
  }
#endif
  SPI.endTransaction();
#if ADC_OVERSAMPLING_BITS > 0
  g_adc_value_buffer = g_adc_value_buffer >> ADC_OVERSAMPLING_BITS;
#endif
  return g_adc_value_buffer;
}

/* ADC reads for TI-ADC121S021
 * this function accumulates into the g_adc_value_buffer each time it is called
 */
inline void adcReadRaw( const uint8_t csPin ) {
  digitalWrite( csPin, LOW );
  g_adc_read_buffer = SPI.transfer16(0);
  digitalWrite( csPin, HIGH );
  //g_adc_read_buffer = g_adc_read_buffer >> 1; // first 3 bits are null, so shift bits down 1 (total of 4) NOTE: the timing diagrams would suggest this, but testing proved otherwise. Not sure why?
  g_adc_read_buffer &= 0b0000111111111111; // mask the unused 4 upper bits. Fix: is this necessary?
  g_adc_value_buffer = g_adc_value_buffer + g_adc_read_buffer;
  //__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");  // may not be necessary, but need 350ns between CS pulses. 12.5ns per MCU clock. was needed on an earlier 10-bit ADC
}


/* debug function for testing the sampling rate under current params.
 * useful for adjusting oversampling bits and code optimizations
 */
void adcTestSPS() {
  Serial.print( "Testing sampling rate at " );
  Serial.print( ADC_OVERSAMPLING_BITS );
  Serial.println( " bits oversampling..." );
  yield();
  uint32_t startTime = micros();
  for (uint16_t i = 0; i<1000; i++) {
    (void)adcRead( CURRENT_LEVEL_CS_PIN );
  }
  uint32_t totalTime = micros() - startTime;
  Serial.print( "SPI 1000 samples done in " );
  Serial.print( totalTime );
  Serial.println( "us" );
  yield();
  startTime = micros();
  for (uint16_t i = 0; i<1000; i++) {
    (void)analogRead(VOLTAGE_LEVEL_PIN);
  }
  totalTime = micros() - startTime;
  Serial.print( "analogRead 1000 samples done in " );
  Serial.print( totalTime );
  Serial.println( "us" );
}


/* sense the line frequency using the pulse interrupts
 */
float_t senseLineHz() {
  g_interruptTimerStart = 0;
  g_interruptTimerEnd = 0;
  attachInterrupt( VOLTAGE_PULSE_PIN, timingHandler, RISING );
  delay( 50 );
  detachInterrupt( VOLTAGE_PULSE_PIN );
  if ( !g_interruptTimerStart || !g_interruptTimerEnd ) return 0;
  float_t cycleUs = g_interruptTimerEnd - g_interruptTimerStart;
  float_t lineHz = 1000000.0 / cycleUs;
  return lineHz;
}


/* senses the value of an adc at 90 degrees from the positive voltage crossing
 */
uint32_t sensePeakValue( const uint8_t csPin ) {
  if (!g_lineHz) return 0;  // must have a valid line hertz sensed
  g_interruptTimerStart = 0;
  long quarterCycleUs = 250000 / g_lineHz;
  yield(); // make sure the wdt has just been reset and chip has serviced other functions
  attachInterrupt( VOLTAGE_PULSE_PIN, timingHandler, RISING );
  while (!g_interruptTimerStart) delay( 0 );
  detachInterrupt( VOLTAGE_PULSE_PIN );
  long delayTime = quarterCycleUs - (micros() - g_interruptTimerStart);
  if (delayTime <= 0) return 0; // make sure we didn't miss our window
  delayMicroseconds( delayTime );
  uint32_t value = adcRead( csPin );
  yield(); // let chip deal with other functions immediately
  return value;
}


/* senses the voltage at 90 degrees from the positive voltage crossing
 */
uint32_t sensePeakVoltage() {
  if (!g_lineHz) return 0;  // must have a valid line hertz sensed
  g_interruptTimerStart = 0;
  long quarterCycleUs = 250000 / g_lineHz;
  yield(); // make sure the wdt has just been reset and chip has serviced other functions
  attachInterrupt( VOLTAGE_PULSE_PIN, timingHandler, RISING );
  while (!g_interruptTimerStart) delay( 0 );
  detachInterrupt( VOLTAGE_PULSE_PIN );
  long delayTime = quarterCycleUs - (micros() - g_interruptTimerStart);
  if (delayTime <= 0) return 0; // make sure we didn't miss our window
  delayMicroseconds( delayTime );
  uint32_t value = analogRead(VOLTAGE_LEVEL_PIN);
  yield(); // let chip deal with other functions immediately
  return value;
}

