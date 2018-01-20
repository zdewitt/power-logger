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
 * and relay control of a single load.
 */


/******** Main Settings ********/

// enable serial messages for hardware testing
//#define MQTT_DEBUG

#if defined(MQTT_DEBUG)
#define DEBUG_MSG(...) sprintf(g_debugMessageBuf, __VA_ARGS__ ); mqttClient.publish(g_debugTopic, g_debugMessageBuf)
#elif defined(DEBUG_ESP_PORT)
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif


// reset the WiFi connection settings stored in EEPROM
//#define RESET_WIFI


#include <FS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <TwoColorLed.h>
#include <SPI.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


/******** Pin Definitions ********/

#define VOLTAGE_PULSE_PIN 13
#define CURRENT_LEVEL_CS_PIN 2
#define VOLTAGE_LEVEL_PIN A0
#define LED_RED_PIN 5
#define LED_GREEN_PIN 4
#define RELAY_ENABLE_PIN 16


/******** Structs ********/

/* Publishing data payload for MQTT
 * All sensing values are sent in a single publish operation so that
 * they update into the database as a single row
 */
typedef struct {
  float realPower;
  float lineFreq;
  float voltsRMS;
  float ampsPeak;
  float powerFactor;
  uint32_t relay;
  uint32_t uptime;
} PublishPayload;

/* RTC memory
 * This struct stores the status of the relay, zero-calibration
 * information, and peak power. In case of a uC reset, these values
 * can be retrieved without interrupting load power. Validity is
 * checked with a 32-bit CRC
 */
typedef struct {
  bool relay;
  uint16_t currentZeroCalibration;
  float_t peakPower;
  uint32_t crc;
} RTCMemory;


/******** Constants ********/

/* ADC factors for current/voltage conversions
 * the voltage factor (V/bit) is determined by the voltage divider
 *    (see schematic) and the ESP8266 analog pin range, 0-1.0V
 * the current factor (A/bit) is determined by the ACS722 range
 *    (-20A,20A) and the bit-depth of the ADC device.
 */
#define ADC_VOLTAGE_FACTOR_10BIT 0.3873416081  // for analogReads, with 488.0/1.27 divider
#define ADC_CURRENT_FACTOR_12BIT .012207031 // for ADC121S021 and ACS722 at 3.3v

// oversampling the current sensor provides better value resolution,
// at the expense of temporal resolution
#define ADC_OVERSAMPLING_BITS 2  // the number of oversampling bits. maximum 2

const float_t ADC_VOLTAGE_FACTOR = ADC_VOLTAGE_FACTOR_10BIT;
const float_t ADC_CURRENT_FACTOR = ADC_CURRENT_FACTOR_12BIT / (1 << ADC_OVERSAMPLING_BITS);

// phase-sync parameters
#define MIN_CYCLE_LEN 14285 // 70Hz
#define MAX_CYCLE_LEN 25000 // 40Hz

const float_t SQRT2 = 1.4142136;

// SPI settings for the ADC121S021
const SPISettings ADC121S021_spi_settings( SPI_CLOCK_DIV8, MSBFIRST, SPI_MODE0 );


/******** Global Variables ********/

volatile uint32_t g_interruptTimerStart = 0;
volatile uint32_t g_interruptTimerEnd = 0;

float_t g_lineHz = 0;
float_t g_voltage = 0;
float_t g_voltagePk = 0;
float_t g_current = 0;
float_t g_powerReal = 0;
float_t g_peakPower = 1.0;
float_t g_powerFactor = 0;

float_t g_lineHzAcc = 0;
float_t g_voltageAcc = 0;
float_t g_currentAcc = 0;
float_t g_powerRealAcc = 0;
float_t g_powerFactorAcc = 0;
uint16_t g_numSamples = 0;

uint16_t g_currentZeroCalibration = 0;

uint32_t g_lastUpdateTime = 0;
uint32_t g_lastPublishTime = 0;

// global buffers for higher-speed transfers
uint32_t g_adc_value_buffer = 0;
uint16_t g_adc_read_buffer = 0;

RTCMemory g_rtcMemory;


/******** I/O ********/

/*  Set up the MQTT interface
 *  For testing we use the Eclipse server with an obscure topic prefix
 */
#define HOST_NAME "iot.eclipse.org"
#define PORT_NUM 1883
#define DEVICE_ID_FMT "010%010u"
#define TOPIC_FMT "pwrctl-ax942dpda/%s/%s"
#define TOPIC_LEN 37
#define SENSE_TOPIC "sense"
#define DEBUG_TOPIC "debug"
#define CONTROL_TOPIC "ctl"
#define POLL_TOPIC "poll"
#define PUBLISH_INTERVAL 60000

void mqttCallback(char* topic, byte* payload, unsigned int length);

WiFiClient wifiClient;
PubSubClient mqttClient(HOST_NAME, PORT_NUM, mqttCallback, wifiClient);

char g_deviceId[14];
char g_controlTopic[TOPIC_LEN];
char g_senseTopic[TOPIC_LEN];
char g_pollTopic[TOPIC_LEN];
#ifdef MQTT_DEBUG
char g_debugTopic[TOPIC_LEN];
char g_debugMessageBuf[256];
#endif


/* device status feedback LED
 * responds to status of internet connection when starting up and modulates
 * the color based on percentage of peak usage
 */
TwoColorLed led(LED_RED_PIN, LED_GREEN_PIN, false);


/******** Function Defines ********/

void ICACHE_RAM_ATTR timingHandler();
void calibrateCurrentSensor();
bool updateLineValues();
bool updatePower();
uint32_t adcRead(const uint8_t csPin);
inline void adcReadRaw(const uint8_t csPin);
void adcTestSPS();
uint32_t calculateCRC32(const uint8_t *data, size_t length);
void updateRTC();
void publishAll();
void reconnectMQTT();


/******** Startup Sequence ********/

void setup() {
  // Read and validate the RTC memory first thing
  ESP.rtcUserMemoryRead(0, (uint32_t*) &g_rtcMemory, sizeof(g_rtcMemory));
  uint32_t crcOfData = calculateCRC32(((uint8_t*) &g_rtcMemory), sizeof(g_rtcMemory) - 4);
  bool rtcValid = (crcOfData == g_rtcMemory.crc);
  
  // Startup pin states
  if (rtcValid) {
    digitalWrite(RELAY_ENABLE_PIN, g_rtcMemory.relay);
    Serial.begin(115200);
  } else {
    Serial.begin(115200);
    digitalWrite(RELAY_ENABLE_PIN, LOW);
    DEBUG_MSG("RTC CRC mismatch: %x %x", crcOfData, g_rtcMemory.crc);
    g_rtcMemory.relay = 0;
    updateRTC();
  }
  pinMode(RELAY_ENABLE_PIN, OUTPUT);
  pinMode(CURRENT_LEVEL_CS_PIN, OUTPUT);
  digitalWrite(CURRENT_LEVEL_CS_PIN, HIGH);
  
  SPI.begin();
  pinMode(VOLTAGE_PULSE_PIN, INPUT_PULLUP); // SPI standard pin, not used
  DEBUG_MSG("\n\n\rBeginning startup...\n\n\r");
  if (rtcValid) {
    DEBUG_MSG("RTC memory is valid...resuming last state\n\n\r");
  }
  DEBUG_MSG("Chip ID: %lu\n\n\r", ESP.getChipId());
  
  
  // solid red to start
  led.setColor(0);
  led.setLevel(1023);
  
  /* calibrate current sensor on startup, unless the RTC memory has
   * the last value stored.
   */
  delay(50);
  if (rtcValid) {
    g_currentZeroCalibration = g_rtcMemory.currentZeroCalibration;
    g_peakPower = g_rtcMemory.peakPower;
  } else {
    calibrateCurrentSensor();
    digitalWrite(RELAY_ENABLE_PIN, HIGH);
    g_rtcMemory.relay = 1;
    g_rtcMemory.currentZeroCalibration = g_currentZeroCalibration;
    updateRTC();
  }
  
  DEBUG_MSG("Starting WiFi...\n\n\r");
  //WiFiManager
  WiFiManager wifiManager;
#ifdef RESET_WIFI
  wifiManager.resetSettings();
#endif
  
  // blink orange while connecting
  led.setBlink(250);
  led.setColor(400);
  
  wifiManager.autoConnect();

  // solid green when connected
  led.setColor(1023);
  led.setBlink(0);
  DEBUG_MSG("WiFi startup complete!\n\r");
  
  //adcTestSPS();  // uncomment this to test the sampling rate at startup
  
  // setup OTA updates
  char otaHostName[22];
  sprintf(otaHostName, "power-ctl-%u", ESP.getChipId());
  ArduinoOTA.setHostname(otaHostName);
  ArduinoOTA.onStart([]() {
    DEBUG_MSG("Start updating ");
    if (ArduinoOTA.getCommand() == U_FLASH) {
      DEBUG_MSG("sketch\n\r");
    } else { // U_SPIFFS
      DEBUG_MSG("filesystem\n\r");
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_MSG("\n\rEnd\n\r");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_MSG("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_MSG("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) { DEBUG_MSG("Auth Failed\n\r"); }
    else if (error == OTA_BEGIN_ERROR) { DEBUG_MSG("Begin Failed\n\r"); }
    else if (error == OTA_CONNECT_ERROR) { DEBUG_MSG("Connect Failed\n\r"); }
    else if (error == OTA_RECEIVE_ERROR) { DEBUG_MSG("Receive Failed\n\r"); }
    else if (error == OTA_END_ERROR) { DEBUG_MSG("End Failed\n\r"); }
  });
  ArduinoOTA.begin();
  DEBUG_MSG("OTA ready\n\r");
  
  // generate device ID and add to topics ****FINISH
  sprintf(g_deviceId, DEVICE_ID_FMT, ESP.getChipId());
  sprintf(g_controlTopic, TOPIC_FMT, g_deviceId, CONTROL_TOPIC);
  sprintf(g_senseTopic, TOPIC_FMT, g_deviceId, SENSE_TOPIC);
  sprintf(g_pollTopic, TOPIC_FMT, g_deviceId, POLL_TOPIC);
  #ifdef MQTT_DEBUG
  sprintf(g_debugTopic, TOPIC_FMT, g_deviceId, DEBUG_TOPIC);
  #endif
  
  DEBUG_MSG("Setup complete!\n\r");
}


/******** Main Loop ********/

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  ArduinoOTA.handle();
  
  if (millis() - g_lastUpdateTime >= 1000) {
    DEBUG_MSG("Running update cycle...\n\r");
    for (int i=0; i<10; i++) {
      if (updateLineValues()) {
        DEBUG_MSG("Line updated after %i tries.\n\r", i);
        break;
      }
      if (i==9) {
        DEBUG_MSG("Line update failed.\n\r");
      }
    }
    if (g_lineHz > 0) {
      for (int i=0; i<10; i++) {
        if (updatePower()) {
          DEBUG_MSG("Power updated after %i tries.\n\r", i);
          break;
        }
        if (i==9) {
          DEBUG_MSG("Power update failed.\n\r");
        }
      }
    }
    g_lastUpdateTime = millis();
  }
  
  if (millis() - g_lastPublishTime >= PUBLISH_INTERVAL && g_lineHz > 0 && g_numSamples > 0) {
    DEBUG_MSG("Publishing to mqtt server\n\r");
    publishAll();
    g_lastPublishTime = millis();
  }
  
  // check the RTC function
  if (Serial.available()) {
    if (Serial.read() == 'r') {
      ESP.reset();
    }
  }
}


/******** MQTT ********/

/* Functions for the MQTT interface
 */
void reconnectMQTT()
{
  while (!mqttClient.connected()) {
    DEBUG_MSG("Establishing mqtt connection...");
    if (mqttClient.connect(g_deviceId)) {
      DEBUG_MSG("MQTT connected\n\r");
      mqttClient.subscribe(g_controlTopic, 1);
    } else {
      DEBUG_MSG("failed! rc=%d\n\r", mqttClient.state());
      delay(5000);
    }
  }
}

void publishAll()
{
  PublishPayload payload = {
    .realPower = g_powerRealAcc/g_numSamples,
    .lineFreq = g_lineHzAcc/g_numSamples,
    .voltsRMS = g_voltageAcc/g_numSamples,
    .ampsPeak = g_currentAcc/g_numSamples,
    .powerFactor = g_powerFactorAcc/g_numSamples,
    .relay = digitalRead(RELAY_ENABLE_PIN),
    .uptime = millis() / 1000
  };
  byte* payloadPtr = reinterpret_cast<byte*>(&payload);
  DEBUG_MSG("MQTT payload: ");
  for (int i=0; i<sizeof(payload); i++) {
    DEBUG_MSG("%X", payloadPtr[i]);
  }
  DEBUG_MSG(": %i\n\r", sizeof(payload));
  mqttClient.publish(
    g_senseTopic,
    payloadPtr,
    sizeof(payload)
  );
  g_numSamples = 0;
  g_lineHzAcc = 0;
  g_voltageAcc = 0;
  g_powerFactorAcc = 0;
  g_powerRealAcc = 0;
  g_currentAcc = 0;
}

void publishPoll()
{
  PublishPayload payload = {
    .realPower = g_powerReal,
    .lineFreq = g_lineHz,
    .voltsRMS = g_voltage,
    .ampsPeak = g_current,
    .powerFactor = g_powerFactor,
    .relay = digitalRead(RELAY_ENABLE_PIN),
    .uptime = millis() / 1000
  };
  byte* payloadPtr = reinterpret_cast<byte*>(&payload);
  DEBUG_MSG("MQTT payload: ");
  for (int i=0; i<sizeof(payload); i++) {
    DEBUG_MSG("%X", payloadPtr[i]);
  }
  DEBUG_MSG(": %i\n\r", sizeof(payload));
  mqttClient.publish(
    g_pollTopic,
    payloadPtr,
    sizeof(payload)
  );
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  if (strcmp(topic, g_controlTopic)==0 && length>0) {
    if (payload[0] == 'p') {
      // Send a poll message
      publishPoll();
    } else if (payload[0] == 'r' && length > 1) {
      // Turn relay on or off
      g_rtcMemory.relay = (payload[1] == '1');
      updateRTC();
      digitalWrite(RELAY_ENABLE_PIN, g_rtcMemory.relay);
      led.setLevel(g_rtcMemory.relay ? 1023 : 0);
    }
  }
}


/******** Calibration ********/

/* sets the no-load value from the current sensor.
 * takes the average of 100 samples, spaced at 10ms
 */
void calibrateCurrentSensor()
{
  uint32_t accumulator = 0;
  for (int i=0; i<100; i++) {
    accumulator += adcRead(CURRENT_LEVEL_CS_PIN);
    delay(10);
  }
  accumulator += 50;
  accumulator = accumulator/100;
  g_currentZeroCalibration = accumulator;
  DEBUG_MSG("Current calibration midpoint set at: %X\n\r", g_currentZeroCalibration);
}


/******** Sensing ********/

/* update the line frequency and voltage global vars
 */
bool updateLineValues()
{
  // sense the line frequency first
  uint32_t startTime = millis();
  g_interruptTimerStart = 0;
  g_interruptTimerEnd = 0;
  attachInterrupt(VOLTAGE_PULSE_PIN, timingHandler, RISING);
  while (!g_interruptTimerEnd && ((millis() - startTime) < 200)) {
    yield();
  }
  detachInterrupt(VOLTAGE_PULSE_PIN);
  if (!g_interruptTimerStart || !g_interruptTimerEnd) return false;
  float_t lineHz = 1000000.0 / (g_interruptTimerEnd - g_interruptTimerStart);
  
  // sense the voltage at 90 degrees from positive voltage crossing
  long quarterCycleUs = 250000 / lineHz;
  long delayTime = quarterCycleUs - (micros() - g_interruptTimerEnd);
  if (delayTime <= 0) return 0; // make sure we didn't miss our window
  delayMicroseconds(delayTime);
  float_t voltagePk = analogRead(VOLTAGE_LEVEL_PIN) * ADC_VOLTAGE_FACTOR;
  float_t voltageRMS = voltagePk/SQRT2;
  yield(); // let chip deal with other functions immediately
  
  // update the global values if the new ones are valid
  DEBUG_MSG("Line sensed: %.2fHz %.2fV\n\r", lineHz, voltageRMS);
  if ((lineHz < 70) && (lineHz > 40) && (voltageRMS < 400) && (voltageRMS > 80)) {
    g_lineHz = lineHz;
    g_voltage = voltageRMS;
    g_voltagePk = voltagePk;
    return true;
  } else {
    g_lineHz = 0;
    g_voltage = 0;
    g_voltagePk = 0;
    return false;
  }
}

/* update the g_powerReal and g_current vars with new values.
 * performs a numerical integration on a single cycle of the
 * voltage and current waveforms. voltage is approximated as a
 * sin wave using the peak voltage and cycle period
 */
bool updatePower()
{
  // get a rising edge from the voltage pulse to synchronize the timing of the voltage waveform
  g_interruptTimerStart = 0;
  g_interruptTimerEnd = 0;
  attachInterrupt( VOLTAGE_PULSE_PIN, timingHandler, RISING );
  while (!g_interruptTimerEnd) {
    yield();
  }
  detachInterrupt(VOLTAGE_PULSE_PIN);
  
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
  lastCurrentADC = adcRead(CURRENT_LEVEL_CS_PIN) - g_currentZeroCalibration;
  while (lastMicros - cycleStartMicros <= cycleLengthMicros) {
    ++numSteps;
    currentMicros = micros();
    currentCurrentADC = adcRead(CURRENT_LEVEL_CS_PIN) - g_currentZeroCalibration;
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
  DEBUG_MSG("Power sensed: %.2fW %.2fW %.2f %.2fA %lu %ld\n\r",
    powerReal, g_peakPower, powerFactor,
    peakCurrentF, maxTimeStep, numSteps);
  
  if (maxTimeStep < 1000 && numSteps > 50) {
    g_powerReal = powerReal;
    g_powerFactor = powerFactor;
    g_current = peakCurrentF;
    g_powerRealAcc += powerReal;
    g_powerFactorAcc += powerFactor;
    g_currentAcc += peakCurrentF;
    g_lineHzAcc += g_lineHz;
    g_voltageAcc += g_voltage;
    g_numSamples++;
    if (g_powerReal > g_peakPower) {
      g_peakPower = g_powerReal;
      g_rtcMemory.peakPower = g_peakPower;
      updateRTC();
    }
    int newColor = 1023 * (1.0 - (g_powerReal / g_peakPower));
    if (g_powerReal < 1.0) newColor=1023;
    led.setColor(newColor);
    return true;
  } else {
    return false;
  }
}


/******** ADC Functions ********/

/* perform a read of an SPI ADC connected to the specified CS pin.
 * returns a raw value with a bit depth higher than the native depth
 * by the number of oversampling bits. 
 */
uint32_t adcRead(const uint8_t csPin)
{
  g_adc_value_buffer = 0;
  g_adc_read_buffer = 0;
  SPI.beginTransaction( ADC121S021_spi_settings );
  
  adcReadRaw(csPin);
#if ADC_OVERSAMPLING_BITS > 0
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
#endif
#if ADC_OVERSAMPLING_BITS > 1
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
  adcReadRaw(csPin);
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
inline void adcReadRaw(const uint8_t csPin)
{
  digitalWrite(csPin, LOW);
  g_adc_read_buffer = SPI.transfer16(0);
  digitalWrite(csPin, HIGH);
  //g_adc_read_buffer = g_adc_read_buffer >> 1; // first 3 bits are null, so shift bits down 1 (total of 4) NOTE: the timing diagrams would suggest this, but testing proved otherwise. Not sure why?
  g_adc_read_buffer &= 0b0000111111111111; // mask the unused 4 upper bits. Fix: is this necessary?
  g_adc_value_buffer = g_adc_value_buffer + g_adc_read_buffer;
}

/* debug function for testing the sampling rate under current params.
 * useful for adjusting oversampling bits and code optimizations
 */
void adcTestSPS()
{
  DEBUG_MSG(
    "Testing sampling rate at %i bits oversampling...",
    ADC_OVERSAMPLING_BITS
  );
  yield();
  uint32_t startTime = micros();
  for (uint16_t i = 0; i<1000; i++) {
    (void)adcRead(CURRENT_LEVEL_CS_PIN);
  }
  uint32_t totalTime = micros() - startTime;
  DEBUG_MSG("SPI 1000 samples done in %ius", totalTime);
  yield();
  startTime = micros();
  for (uint16_t i = 0; i<1000; i++) {
    (void)analogRead(VOLTAGE_LEVEL_PIN);
  }
  totalTime = micros() - startTime;
  DEBUG_MSG("analogRead 1000 samples done in %ius", totalTime);
}


/******** Interrupt Routines ********/

/* handles interrupts from a pin change and records up to two timestamps
 */
void ICACHE_RAM_ATTR timingHandler()
{
  uint32_t length = micros() - g_interruptTimerStart;
  if (!g_interruptTimerStart || length < MIN_CYCLE_LEN || length > MAX_CYCLE_LEN) {
    g_interruptTimerStart = micros();
  } else if (!g_interruptTimerEnd) {
    g_interruptTimerEnd = micros();
  }
}


/******** Utilities ********/

/* Calculate the 32-bit crc of an arbitrary payload
 */
uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

/* Update the values in the RTC memory from the global buffer
 */
void updateRTC()
{
  g_rtcMemory.crc = calculateCRC32(
    (uint8_t*) &g_rtcMemory,
    sizeof(g_rtcMemory) - 4
  );
  ESP.rtcUserMemoryWrite(
    0,
    (uint32_t*) &g_rtcMemory,
    sizeof(g_rtcMemory)
  );
}

