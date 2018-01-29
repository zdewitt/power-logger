# power-logger

ESP8266 Arduino project for a miniature WiFi enabled energy usage logger

This repo contains the operational code for Pwrctl hardware devices. Pwrctl devices are small and simple
high-frequency energy data sensing devices that connect in-line to a device and update to a server in 
real-time. They are powered from the line voltage so no batteries or other power supply is needed for
operation. Optional on relay-enabled devices is a control relay that can be used to control the load.

Pwrctl devices sense watts, volts, amps, Hz, and power factor in real-time and can be configured to publish
average values on a set interval to a server. High-frequency waveform data (~8ksps) can also be obtained and
written to a server, or polled in real-time.

## MQTT Interface

Pwrctl devices communicate through an MQTT broker to send data and receive commands. Currently the devices
are set up to use the public server at `iot.eclipse.org:1883`, but this can be changed in the code if a
more secure server is needed. The root topic for all devices is *pwrctl-ax942dpda*, with devices publishing
and receiving on topics of the form `pwrctl-ax942dpda/(device_id)/(sub)` where `device_id` is the serial
number printed on the device and `sub` is the sub topic.

The control and messaging interface for Pwrctl devices consists of the following MQTT sub topics:

* ### /ctl
  
  For sending control messages to the device. The currently supported commands are:
  
  * **_p_**  
    Keep the polling alive for another two minutes. Widgets that get the current state of the device use this
    message to keep the polling interval alive. The device will send a message every 5 seconds on the /poll
    topic.
  
  * **_r0/r1_**  
    (Relay-enabled devices only) Turn the relay on (r1) or off (r0)
  
  * **_c_**  
    Calibrate the zero-point of the current sensor. On relay-enabled devices this command will disable the
    relay during calibration. For devices without, **you must disconnect all loads before calibrating.**
  
  * **_w0/w1_**  
    Enable/disable waveform logging to the server. w1 enables, w0 disables. When enabled, the device
    will send a current/voltage waveform at every logging interval.
  
  * **_i_**_(seconds)_  
    Set the logging interval in seconds, up to 9999. i0 will disable automatic logging.
  
  * **_u_**  
    Run a waveform and power/line values update (for debugging purposes)

* ### /sense
  
  For logging the average power, line frequency, RMS voltage, and power factor to the server. At each logging
  interval, the device will send the average of all these values for the prior interval time. The payload
  is a C-struct of the format:
  
  | Size | Value |
  | --- | --- |
  | `float32` | `P_real` |
  | `float32` | `Hz` |
  | `float32` | `V_rms` |
  | `float32` | `pf` |

* ### /wave
  
  For logging the current/voltage waveform data to the server. At each logging interval, if waveform logging
  is enabled, the device will send a `WavePacket` with the following structure:
  
  | Size | Value |
  | --- | --- |
  | `uint32` | `cycleLength` (microseconds) |
  | `uint32` | `numSamples` |
  | `uint32` | `midpoint` (microseconds) |
  | `WaveDataPoint[numSamples]` | (data) |
  
  Each `WaveDataPoint` sample in the packet is of the form:
  
  | Size | Value |
  | --- | --- |
  | `uint32` | `frameMicros` |
  | `int16` | `I_inst` |
  | `int16` | `V_inst` |
  
  In each cycle frame, `cycleLength` is the total length of the sampled waveform from positive zero-crossing
  to positive zero-crossing, `numSamples` denotes the number of samples taken, and `midpoint` is the time of
  the negative zero-crossing. Each data sample consists of `frameMicros` when the sample was taken, `I_inst`
  and `V_inst` which are the instantaneous current and voltage readings in raw ADC format. To convert into
  real volts and amps the following conversion factors are useful:
  ```
  V = V_inst * 0.3873416081
  I = I_inst * 0.012207031
  ```

* ### /poll
  
  When polling has been kept alive with a **_p_** command, the device will publish the current state on this
  topic every 5 seconds. Webpage widgets use this to show the current values in the browser. The message is
  of the format:
  
  | Size | Value |
  | --- | --- |
  | `float32` | `P_real` |
  | `float32` | `Hz` |
  | `float32` | `V_rms` |
  | `float32` | `I_rms` |
  | `float32` | `pf` |
  | `uint32` | `relay` |
  | `uint32` | `uptime` (milliseconds) |
  | `WavePacket` | (waveform data) |
  
  The values are the same as in the **_/sense_** topic with the addition of RMS current, the state of the relay
  (1 on/0 off), the uptime since the device last reset, and a `WavePacket` of the form sent on the **_/wave_**
  topic.
  
* ### /debug
  
  When `MQTT_DEBUG` is defined, debug messages will be sent over this MQTT topic. Messages may be broken up into
  many chunks, but will be terminated with `\n\r` chars.
