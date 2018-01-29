/*
 * PwrCtlRevA.ino
 * 
 * Copyright 2018 Zoltan DeWitt
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * Requires:
 *      ESP8266 Arduino: https://github.com/esp8266/Arduino
 *      ESP8266 WiFiManager: https://github.com/tzapu/WiFiManager
 *      ESP8266 TwoColorLed: https://github.com/zdewitt/TwoColorLed
 *      PubSubClient: https://pubsubclient.knolleary.net
 *            NOTE: This sketch needs MQTT payloads that are larger
 *            than the default in the library. Due to the way
 *            Arduino does imports, this must be changed in the .h file:
 *                #define MQTT_MAX_PACKET_SIZE 2048
 * 
 * This is a prototype sketch for a Power Logger device
 * It will connect over WiFi to the Enerbits host and log power usage
 * and relay control of a single load.
 */


/******** Main Settings ********/

// Use if the board has the relay circuit populated
//#define ENABLE_RELAY

// enable serial messages for hardware testing
#define MQTT_DEBUG

// for old versions of the hardware
//#define OLD_PINOUT

#if defined(MQTT_DEBUG)
#define DEBUG_MSG(...) sprintf(g_debugMessageBuf, __VA_ARGS__ ); mqttClient.publish(g_debugTopic, g_debugMessageBuf)
#elif defined(DEBUG_ESP_PORT)
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif


// reset the WiFi connection settings
//#define RESET_WIFI

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <TwoColorLed.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>


/******** Pin Definitions ********/

#define VOLTAGE_PULSE_PIN 13
#define CURRENT_LEVEL_CS_PIN 2
#define VOLTAGE_LEVEL_PIN A0
#define LED_RED_PIN 5
#ifdef OLD_PINOUT
  #define LED_GREEN_PIN 4
  #define RELAY_ENABLE_PIN 16
#else
  #define LED_GREEN_PIN 16
  #define RELAY_ENABLE_PIN 4
#endif


/******** Constants ********/

#define UPDATE_INTERVAL 1000
#define POLL_INTERVAL 5000
#define POLL_KEEP_ALIVE_TIMEOUT 120000
#define MAX_WAVEFORM_SAMPLES 200

/* ADC factors for current/voltage conversions
 * the voltage factor (V/bit) is determined by the voltage divider
 *    (see schematic) and the ESP8266 analog pin range, 0-1.0V
 * the current factor (A/bit) is determined by the ACS722 range
 *    (-20A,20A) and the bit-depth of the ADC device.
 */
// for analogReads, with 488.0/1.27 divider
#define ADC_VOLTAGE_FACTOR_10BIT 0.3873416081
// for ADC121S021 and ACS722 at 3.3v
#define ADC_CURRENT_FACTOR_12BIT .012207031

// Oversampling the current sensor provides better value resolution,
// at the expense of temporal resolution. Maximum 2 bits
#define ADC_OVERSAMPLING_BITS 0

const float_t ADC_VOLTAGE_FACTOR = ADC_VOLTAGE_FACTOR_10BIT;
const float_t ADC_CURRENT_FACTOR =
  ADC_CURRENT_FACTOR_12BIT / (1 << ADC_OVERSAMPLING_BITS);

// Sampling latency to synchronize the timing properly
#define SAMPLING_LATENCY_MICROS 50

// phase-sync parameters
#define MIN_CYCLE_LEN 14285 // 70Hz
#define MAX_CYCLE_LEN 25000 // 40Hz

const float_t SQRT2 = 1.4142136;

// SPI settings for the ADC121S021
const SPISettings ADC121S021_spi_settings(
  SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0
);

// MQTT settings
#define HOST_NAME "iot.eclipse.org"
#define PORT_NUM 1883
#ifdef ENABLE_RELAY
  #define DEVICE_ID_FMT "010%010u"
#else
  #define DEVICE_ID_FMT "020%010u"
#endif
#define TOPIC_FMT "pwrctl-ax942dpda/%s/%s"
#define TOPIC_LEN 37
#define SENSE_TOPIC "sense"
#define DEBUG_TOPIC "debug"
#define CONTROL_TOPIC "ctl"
#define POLL_TOPIC "poll"
#define WAVEFORM_TOPIC "wave"


/******** Structs ********/

/* Publishing data payload for MQTT
 * All sensing values are sent in a single publish operation so that
 * they update into the database as a single row
 */
typedef struct SensePayload {
  float realPower;
  float lineFreq;
  float voltsRMS;
  float powerFactor;
} SensePayload;

/* Waveform data point
 * Single data point from a waveform
 */
 typedef struct WaveDataPoint {
  uint32_t frameMicros;
  int16_t I_inst;
  int16_t V_inst;
} WaveDataPoint;

/* Waveform packet
 * For building the packet to analyze and send across MQTT
 */
typedef struct WavePacket {
  uint32_t cycleLength;
  uint32_t numSamples;
  uint32_t midpoint;
  WaveDataPoint data[MAX_WAVEFORM_SAMPLES];
} WavePacket;

/* Sensing data struct
 * This stores all the sampled and calculated values from the last
 * sensing and calculation update. This is sent over mqtt when polling
 * is active.
 */
typedef struct SenseData {
  float realPower;
  float lineFreq;
  float voltsRMS;
  float ampsRMS;
  float powerFactor;
  uint32_t relay;
  uint32_t uptime;
  WavePacket wavePacket;
} SenseData;

/* RTC memory
 * This struct stores the status of the relay and peak power.
 * In case of a uC reset, these values
 * can be retrieved without interrupting load power. Validity is
 * checked with a 32-bit CRC. If the relay is not populated, this
 * only stores the peak power.
 */
typedef struct RTCMemory {
#ifdef ENABLE_RELAY
  bool relay;
#endif
  float_t peakPower;
  uint32_t crc;
} RTCMemory;

/* EEPROM memory
 * This struct stores the current sensor calibration and publishing
 * inverval in seconds. The data is validated with a 32-bit crc.
 */
typedef struct EEPROMMemory {
  uint16_t currentZeroCalibration;
  uint16_t publishInterval;
  bool publishWaveform;
  uint32_t crc;
} EEPROMMemory;

/* Timeout
 * Convenience class for making a simple timeout
 */
class TimeoutMicros {
  private:
    uint32_t _duration;
    uint32_t _start;
  public:
    TimeoutMicros(uint32_t duration) {
      _duration = duration;
      _start = micros();
    };
    bool expired() {
      return micros() - _start > _duration;
    };
};


/******** Globals ********/

void ICACHE_RAM_ATTR timingHandler();
void calibrateCurrentSensor();
//bool updateLineValues();
bool updateWaveform();
//bool updatePower();
bool calcLineValues();
int16_t interpVoltage(int32_t t);
void adcRead();
void adcReadRaw();
void adcTestSPS();
uint32_t calculateCRC32(const uint8_t *data, size_t length);
void updateRTC();
void publishSense();
void publishPoll();
void publishWaveform();
void connectMQTT();
bool readEEPROM();
void saveEEPROM();
void startOTA();
void mqttCallback(char* topic, byte* payload, unsigned int length);

volatile uint32_t g_interruptTimerStart = 0;
volatile uint32_t g_interruptTimerMid = 0;
volatile uint32_t g_interruptTimerEnd = 0;
float_t g_lineHzAcc = 0;
float_t g_voltageAcc = 0;
float_t g_powerRealAcc = 0;
float_t g_powerFactorAcc = 0;
uint32_t g_numSamples = 0;
uint32_t g_lastUpdateTime = 0;
uint32_t g_lastPublishTime = 0;
uint32_t g_lastPollTime = 0;
uint32_t g_lastPollKeepAlive = 0;
uint32_t g_adcValueBuffer = 0;
uint16_t g_adcReadBuffer = 0;
RTCMemory g_rtcMemory;
EEPROMMemory g_em;
WiFiClient wifiClient;
PubSubClient mqttClient(HOST_NAME, PORT_NUM, mqttCallback, wifiClient);
char g_deviceId[14];
char g_controlTopic[TOPIC_LEN];
char g_senseTopic[TOPIC_LEN];
char g_pollTopic[TOPIC_LEN];
char g_waveformTopic[TOPIC_LEN];
#ifdef MQTT_DEBUG
char g_debugTopic[TOPIC_LEN];
char g_debugMessageBuf[256];
#endif
SenseData g_senseData;
TwoColorLed led(LED_RED_PIN, LED_GREEN_PIN, false);


/******** Startup Sequence ********/

void setup() {
  // Read and validate the RTC memory first thing
  ESP.rtcUserMemoryRead(
    0, (uint32_t*) &g_rtcMemory, sizeof(g_rtcMemory)
  );
  uint32_t crcOfData = calculateCRC32(
    ((uint8_t*) &g_rtcMemory), sizeof(g_rtcMemory) - 4
  );
  bool rtcValid = (crcOfData == g_rtcMemory.crc);
  
  if (crcOfData == g_rtcMemory.crc) {
    #ifdef ENABLE_RELAY
    // Deal with relay control at startup
    digitalWrite(RELAY_ENABLE_PIN, g_rtcMemory.relay);
    g_senseData.relay = g_rtcMemory.relay;
    #endif
  } else {
    #ifdef ENABLE_RELAY
    digitalWrite(RELAY_ENABLE_PIN, LOW);
    g_rtcMemory.relay = 0;
    #endif
    g_rtcMemory.peakPower = 1.0;
    g_senseData.relay = 0;
    updateRTC();
  }
  #ifdef ENABLE_RELAY
  pinMode(RELAY_ENABLE_PIN, OUTPUT);
  #endif
  Serial.begin(115200);

  pinMode(CURRENT_LEVEL_CS_PIN, OUTPUT);
  digitalWrite(CURRENT_LEVEL_CS_PIN, HIGH);
  
  SPI.begin();
  pinMode(VOLTAGE_PULSE_PIN, INPUT_PULLUP); // SPI pin, not used
  DEBUG_MSG("\n\n\rBeginning startup...\n\n\r");
  
  // generate device ID and add to topics
  sprintf(g_deviceId, DEVICE_ID_FMT, ESP.getChipId());
  sprintf(g_controlTopic, TOPIC_FMT, g_deviceId, CONTROL_TOPIC);
  sprintf(g_senseTopic, TOPIC_FMT, g_deviceId, SENSE_TOPIC);
  sprintf(g_pollTopic, TOPIC_FMT, g_deviceId, POLL_TOPIC);
  sprintf(g_waveformTopic, TOPIC_FMT, g_deviceId, WAVEFORM_TOPIC);
  #ifdef MQTT_DEBUG
  sprintf(g_debugTopic, TOPIC_FMT, g_deviceId, DEBUG_TOPIC);
  #endif
  Serial.printf("Device ID: %s\n\n\r", g_deviceId);
  
  
  // solid red to start
  led.setColor(0);
  led.setLevel(1023);
  
  
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
  
  connectMQTT();

  // Load data from EEPROM and check if it's valid
  EEPROM.begin(sizeof(EEPROMMemory));
  if (readEEPROM()) {
    DEBUG_MSG("EEPROM valid\n\r");
  } else {
    // Reset to default values
    DEBUG_MSG("EEPROM crc mismatch\n\r");
    g_em.currentZeroCalibration = 2^(12+ADC_OVERSAMPLING_BITS)/2;
    g_em.publishInterval = 0;
    g_em.publishWaveform = false;
  }

  //adcTestSPS(); // uncomment this to test the sampling rate at startup
  
  startOTA();
  
  DEBUG_MSG("Setup complete!\n\r");
}


/******** Main Loop ********/

void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();
  ArduinoOTA.handle();
  
  uint32_t currentTime = millis();
  #ifdef UPDATE_INTERVAL
  if (currentTime - g_lastUpdateTime >= UPDATE_INTERVAL) {
    for (int i=0; i<10; i++) {
      bool valid = updateWaveform();
      valid &= calcLineValues();
      if (valid) {
        if (i>0) {
          DEBUG_MSG("Update retries: %i\n\r", i);
        }
        break;
      } else if (i==9) {
        DEBUG_MSG("Update failed.\n\r");
      }
    }
    g_lastUpdateTime = currentTime;
  }
  #endif
  
  if (
    g_em.publishInterval
    && currentTime - g_lastPublishTime >=
      (uint32_t)g_em.publishInterval * 1000
  ) {
    if (g_numSamples > 0) {
      publishSense();
      if (g_em.publishWaveform) {
        publishWaveform();
      }
    }
    g_lastPublishTime = currentTime;
  }
  
  if (g_lastPollTime && currentTime - g_lastPollTime >= POLL_INTERVAL) {
    publishPoll();
    if (currentTime - g_lastPollKeepAlive < POLL_KEEP_ALIVE_TIMEOUT) {
      g_lastPollTime = currentTime;
      if (g_lastPollTime == 0) g_lastPollTime = 1;
    } else {
      g_lastPollTime = 0;
      g_lastPollKeepAlive = 0;
    }
  }
}


/******** MQTT ********/

/* Functions for the MQTT interface
 */
void connectMQTT()
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

void publishSense()
{
  SensePayload payload = {
    .realPower = g_powerRealAcc/g_numSamples,
    .lineFreq = g_lineHzAcc/g_numSamples,
    .voltsRMS = g_voltageAcc/g_numSamples,
    .powerFactor = g_powerFactorAcc/g_numSamples,
  };
  byte* payloadPtr = reinterpret_cast<byte*>(&payload);
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
  DEBUG_MSG("Sense %i\n\r", sizeof(payload));
}

void publishPoll()
{
  uint16_t length = sizeof(SenseData) - sizeof(WaveDataPoint)
    * (MAX_WAVEFORM_SAMPLES - g_senseData.wavePacket.numSamples);
  mqttClient.publish(
    g_pollTopic,
    reinterpret_cast<byte*>(&g_senseData),
    length
  );
  DEBUG_MSG("Poll %i\n\r", length);
}

void publishWaveform()
{
  uint16_t length = sizeof(WavePacket) - sizeof(WaveDataPoint)
    * (MAX_WAVEFORM_SAMPLES - g_senseData.wavePacket.numSamples);
  mqttClient.publish(
    g_waveformTopic,
    reinterpret_cast<byte*>(&g_senseData.wavePacket),
    length
  );
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  if (strcmp(topic, g_controlTopic)==0 && length>0) {
    if (payload[0] == 'p') {
      // Keep polling alive
      g_lastPollKeepAlive = millis();
      if (!g_lastPollTime) {
        g_lastPollTime = g_lastPollKeepAlive - POLL_INTERVAL;
      }
    #ifdef ENABLE_RELAY
    } else if (payload[0] == 'r' && length > 1) {
      // Turn relay on or off
      g_rtcMemory.relay = (payload[1] == '1');
      g_senseData.relay = g_rtcMemory.relay;
      updateRTC();
      digitalWrite(RELAY_ENABLE_PIN, g_rtcMemory.relay);
      led.setLevel(g_rtcMemory.relay ? 1023 : 0);
    #endif
    } else if (payload[0] == 'c') {
      // Calibrate current sensor and save to eeprom
      #ifdef ENABLE_RELAY
      digitalWrite(RELAY_ENABLE_PIN, LOW);
      delay(500);
      #endif
      calibrateCurrentSensor();
      #ifdef ENABLE_RELAY
      digitalWrite(RELAY_ENABLE_PIN, g_rtcMemory.relay);
      #endif
      saveEEPROM();
    } else if (payload[0] == 'w') {
      if (length == 1) {
        // Send a waveform
        publishWaveform();
      } else {
        // toggle waveform interval publishing
        g_em.publishWaveform = payload[1] == '1';
        saveEEPROM();
      }
    } else if (payload[0] == 'i' && length > 1 && length <= 5) {
      // Set publish interval, up to 9999 seconds
      char s[5];
      for (int i=1; i<length; i++) {
        s[i-1] = payload[i];
      }
      s[length-1] = 0;
      g_em.publishInterval = atoi(s);
      saveEEPROM();
      // Reset publishing vars
      g_lastUpdateTime = millis();
      g_numSamples = 0;
      g_lineHzAcc = 0;
      g_voltageAcc = 0;
      g_powerFactorAcc = 0;
      g_powerRealAcc = 0;
      DEBUG_MSG("Interval %is\n\r", g_em.publishInterval);
    } else if (payload[0] == 'u') {
      // run an update, for debugging
      bool valid = updateWaveform();
      DEBUG_MSG("update %i\n\r", valid);
      delay(100);
      calcLineValues();
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
    adcRead();
    accumulator += g_adcValueBuffer;
    delay(10);
  }
  accumulator += 50;
  accumulator = accumulator/100;
  g_em.currentZeroCalibration = accumulator;
  DEBUG_MSG(
    "Midpoint calibrated at: %x\n\r",
    g_em.currentZeroCalibration
  );
}


/******** Sensing ********/

/* Collects a full, valid waveform into the WavePacket buffer
 */
bool updateWaveform()
{
  uint8_t tries = 0;
  float_t dutyCycle;
  bool valid = false;
  yield();
  
  // Loop until valid (max 10)
  do {
    g_senseData.wavePacket.numSamples = 0;
    g_senseData.wavePacket.midpoint = 0;
    TimeoutMicros timeout(50000);
    g_interruptTimerStart = 0;
    g_interruptTimerMid = 0;
    g_interruptTimerEnd = 0;
    attachInterrupt(VOLTAGE_PULSE_PIN, timingHandler, CHANGE);
    
    // Wait for the beginning of a cycle
    while (!g_interruptTimerStart && !timeout.expired()) {}
    
    // Write the data
    while (
      !g_interruptTimerEnd
      && !timeout.expired()
      && g_senseData.wavePacket.numSamples < MAX_WAVEFORM_SAMPLES
    ) {
      WaveDataPoint* dataPoint =
        &g_senseData.wavePacket.data[g_senseData.wavePacket.numSamples];
      dataPoint->V_inst = analogRead(VOLTAGE_LEVEL_PIN);
      adcRead();
      dataPoint->I_inst =
        (int16_t)g_adcValueBuffer-g_em.currentZeroCalibration;
      dataPoint->frameMicros =
        micros() - g_interruptTimerStart - SAMPLING_LATENCY_MICROS;
      g_senseData.wavePacket.numSamples++;
    }
    detachInterrupt(VOLTAGE_PULSE_PIN);
    g_senseData.wavePacket.cycleLength =
      g_interruptTimerEnd - g_interruptTimerStart;
    g_senseData.wavePacket.midpoint =
      g_interruptTimerMid - g_interruptTimerStart;
    tries++;
    
    dutyCycle = (float_t)g_senseData.wavePacket.midpoint
      / (float_t)g_senseData.wavePacket.cycleLength;
    valid =
      MIN_CYCLE_LEN < g_senseData.wavePacket.cycleLength
      && g_senseData.wavePacket.cycleLength < MAX_CYCLE_LEN
      && 0.45 < dutyCycle && dutyCycle < 0.55;
    yield();
  } while (!valid && tries < 10);
  return valid;
}

/* Calculates the Hz, RMS voltage, RMS current, real power,
 * and power factor values from the last stored WavePacket
 * and updates the global vars. Also rewrites the interpolated
 * data into the packet where volts are negative.
 */
bool calcLineValues()
{
  uint16_t sample = 0;
  uint16_t voltSamples = 0;
  int32_t midpoint = g_senseData.wavePacket.cycleLength / 2;
  float_t sumVSq = 0;
  float_t sumISq = 0;
  float_t sumP = 0;
  float_t V_rms;
  float_t I_rms;
  float_t P_real;
  float_t P_app;
  float_t pf;
  bool voltsPos;
  
  while (sample < g_senseData.wavePacket.numSamples) {
    WaveDataPoint* dataPoint = &g_senseData.wavePacket.data[sample];
    voltsPos =
      dataPoint->frameMicros < midpoint
      || dataPoint->frameMicros > g_senseData.wavePacket.cycleLength;
    
    sumISq +=
      pow(dataPoint->I_inst * ADC_CURRENT_FACTOR, 2);
    if (voltsPos) {
      // use sampled data
      sumVSq +=
        pow(dataPoint->V_inst * ADC_VOLTAGE_FACTOR, 2);
      voltSamples++;
      sumP +=
        dataPoint->I_inst * ADC_CURRENT_FACTOR
        * dataPoint->V_inst * ADC_VOLTAGE_FACTOR;
    } else {
      // use negative interpolated data from first half
      dataPoint->V_inst = -interpVoltage(
        (int32_t)dataPoint->frameMicros - (int32_t)midpoint
      );
      sumP +=
        dataPoint->V_inst * ADC_VOLTAGE_FACTOR
        * dataPoint->I_inst * ADC_CURRENT_FACTOR;
    }
    sample++;
  }
  
  V_rms = sqrt(sumVSq / voltSamples);
  I_rms = sqrt(sumISq / g_senseData.wavePacket.numSamples);
  P_real = sumP / g_senseData.wavePacket.numSamples;
  P_app = V_rms * I_rms;
  pf = P_real / P_app;
  
  if (80 < V_rms && V_rms < 400) {
    // Update instantaneous values
    g_senseData.lineFreq = 1000000.0 / g_senseData.wavePacket.cycleLength;
    g_senseData.voltsRMS = V_rms;
    g_senseData.ampsRMS = I_rms;
    g_senseData.realPower = P_real;
    g_senseData.powerFactor = pf;
    // Add to publishing accumulators
    g_numSamples++;
    g_lineHzAcc += g_senseData.lineFreq;
    g_voltageAcc += g_senseData.voltsRMS;
    g_powerRealAcc += g_senseData.realPower;
    g_powerFactorAcc += g_senseData.powerFactor;
    return true;
  } else {
    g_senseData.lineFreq = 0;
    g_senseData.voltsRMS = 0;
    g_senseData.ampsRMS = 0;
    g_senseData.realPower = 0;
    g_senseData.powerFactor = 0;
    DEBUG_MSG("%f %f %f %f\n\r", V_rms, I_rms, P_real, pf);
    return false;
  }
}

/* Perform an interpolation from the closest two voltage values
 * in the stored waveform
 */
int16_t interpVoltage(int32_t t)
{
  uint16_t sample = 0;
  int32_t v1;
  int32_t v2;
  int32_t t1;
  int32_t t2;
  do {
    if (t<g_senseData.wavePacket.data[sample].frameMicros) break;
    sample++;
  } while (sample < g_senseData.wavePacket.numSamples);
  if (sample == 0) {
    v1 = 0;
    v2 = g_senseData.wavePacket.data[0].V_inst;
    t1 = 0;
    t2 = g_senseData.wavePacket.data[0].frameMicros;
  } else if (sample == g_senseData.wavePacket.numSamples) {
    v1 = g_senseData.wavePacket.data[
      g_senseData.wavePacket.numSamples-1].V_inst;
    v2 = 0;
    t1 = g_senseData.wavePacket.data[
      g_senseData.wavePacket.numSamples-1].frameMicros;
    t2 = g_senseData.wavePacket.cycleLength;
  } else {
    v1 = g_senseData.wavePacket.data[sample-1].V_inst;
    v2 = g_senseData.wavePacket.data[sample].V_inst;
    t1 = g_senseData.wavePacket.data[sample-1].frameMicros;
    t2 = g_senseData.wavePacket.data[sample].frameMicros;
  }
  return (v2-v1)*(t-t1)/(t2-t1) + v1;
}


/******** ADC Functions ********/

/* perform a read of an SPI ADC connected to the specified CS pin.
 * returns a raw value with a bit depth higher than the native depth
 * by the number of oversampling bits. 
 */
void adcRead()
{
  g_adcValueBuffer = 0;
  g_adcReadBuffer = 0;
  SPI.beginTransaction(ADC121S021_spi_settings);
  
  adcReadRaw();
#if ADC_OVERSAMPLING_BITS > 0
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
#endif
#if ADC_OVERSAMPLING_BITS > 1
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
  adcReadRaw();
#endif
  SPI.endTransaction();
#if ADC_OVERSAMPLING_BITS > 0
  g_adcValueBuffer = g_adcValueBuffer >> ADC_OVERSAMPLING_BITS;
#endif
}

/* ADC reads for TI-ADC121S021
 * this function accumulates into the g_adcValueBuffer each
 * time it is called
 */
void adcReadRaw()
{
  digitalWrite(CURRENT_LEVEL_CS_PIN, LOW);
  g_adcReadBuffer = SPI.transfer16(0);
  digitalWrite(CURRENT_LEVEL_CS_PIN, HIGH);
   /* first 3 bits are null, so shift bits down 1 (total of 4)
    * NOTE: the timing diagrams would suggest this, but testing
    * proved otherwise. Not sure why?
    */
  //g_adcReadBuffer = g_adcReadBuffer >> 1;
  // mask the unused 4 upper bits. Fix: is this necessary?
  g_adcReadBuffer &= 0b0000111111111111; 
  g_adcValueBuffer = g_adcValueBuffer + g_adcReadBuffer;
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
    adcRead();
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

/* Handles interrupts from a pin change. Records the beginning,
 * middle, and end of a waveform cycle.
 */
void ICACHE_RAM_ATTR timingHandler()
{
  if (digitalRead(VOLTAGE_PULSE_PIN)) {
    if (!g_interruptTimerStart) {
      g_interruptTimerStart = micros();
    } else if (!g_interruptTimerEnd && g_interruptTimerMid) {
      g_interruptTimerEnd = micros();
    }
  } else {
    if (g_interruptTimerStart && !g_interruptTimerMid) {
      g_interruptTimerMid = micros();
    }
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

/* Read values from the EEPROM
 */
bool readEEPROM() {
  byte* emPtr = reinterpret_cast<byte*>(&g_em);
  for (int a=0; a<sizeof(EEPROMMemory); a++) {
    emPtr[a] = EEPROM.read(a);
  }
  uint32_t eepromCrc = calculateCRC32(emPtr, sizeof(EEPROMMemory) - 4);
  return (eepromCrc == g_em.crc);
}

/* Save values to the EEPROM
 */
void saveEEPROM() {
  byte* emPtr = reinterpret_cast<byte*>(&g_em);
  g_em.crc = calculateCRC32(emPtr, sizeof(EEPROMMemory) - 4);
  for (int a=0; a<sizeof(EEPROMMemory); a++) {
    EEPROM.write(a, emPtr[a]);
  }
  EEPROM.commit();
}

/* Start OTA updates
 */
void startOTA() {
  char otaHostName[22];
  sprintf(otaHostName, "pwrctl-%s", g_deviceId);
  ArduinoOTA.setHostname(otaHostName);
  ArduinoOTA.onStart([]() {
    DEBUG_MSG("Start updating ");
    if (ArduinoOTA.getCommand() == U_FLASH) {
      DEBUG_MSG("sketch\n\r");
    } else { // U_SPIFFS
      DEBUG_MSG("filesystem\n\r");
    }

    /* NOTE: if updating SPIFFS this would be the place to 
     * unmount SPIFFS using SPIFFS.end()
     */
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_MSG("\n\rEnd\n\r");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_MSG("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_MSG("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      DEBUG_MSG("Auth Failed\n\r");
    }
    else if (error == OTA_BEGIN_ERROR) {
      DEBUG_MSG("Begin Failed\n\r");
    }
    else if (error == OTA_CONNECT_ERROR) {
      DEBUG_MSG("Connect Failed\n\r");
    }
    else if (error == OTA_RECEIVE_ERROR) {
      DEBUG_MSG("Receive Failed\n\r");
    }
    else if (error == OTA_END_ERROR) {
      DEBUG_MSG("End Failed\n\r");
    }
  });
  ArduinoOTA.begin();
  DEBUG_MSG("OTA ready\n\r");
}
