//----------------------------------CONFIGURATION, DO NOT CHANGE----------------------------------//
#include <MAX3010x.h>
#include "filters.h"

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_3200SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

//------------------------------------------------------------------------------------------------//

//----- Averaging Parameters -----//
const bool kEnableAveraging = true;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

// Filter Instances
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;

//----- Define your LEDs here -----//
#define LED1 A0
#define LED2 A1
#define LED3 A2
#define LED4 A3
#define LED5 A4

//----- Define your variables here -----//

// BPM as global var
int bpm;
int bpm_out;

int led1state, led2state, led3state, led4state, led5state = 0;
long currentMillis, prevMillis = 0;


//----- Setup Function (runs once) -----//
void setup() {


  Serial.begin(115200);  //baud rate
  delay(100);
  Serial.println("Serial start");
  delay(100);
  Serial.println("LED test");
  analogWrite(LED1, 255);
  analogWrite(LED2, 255);
  analogWrite(LED3, 255);
  analogWrite(LED4, 255);
  analogWrite(LED5, 255);
  delay(1000);
  analogWrite(LED1, 0);
  analogWrite(LED2, 0);
  analogWrite(LED3, 0);
  analogWrite(LED4, 0);
  analogWrite(LED5, 0);

  //Sensor initialisation
  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    while (1)
      ;
  }
}

//----- Main Function (repeats forever) -----//
void loop() {

  bpm = readBPM();
  // delay(50);
  currentMillis = millis();
  if (currentMillis - prevMillis > 1000) {
    if (bpm == -1) {
      Serial.println("No BPM detected.");
    } else {
      Serial.println(bpm);
    }
    Serial.print(">>> The function output is:");
    Serial.println(bpm);
    prevMillis = currentMillis;
  }

  //----- Define your LEDs here -----//
  int val = 10;

  if (bpm > 150) {
    led1state = val;
    led2state = val;
    led3state = val;
    led4state = val;
    led5state = val;
  } else if (bpm >= 95) {
    led1state = val;
    led2state = val;
    led3state = val;
    led4state = val;
    led5state = 0;
  } else if (bpm >= 85) {
    led1state = val;
    led2state = val;
    led3state = val;
    led4state = 0;
    led5state = 0;
  } else if (bpm >= 80) {
    led1state = val;
    led2state = val;
    led3state = 0;
    led4state = 0;
    led5state = 0;
  } else if (bpm >= 50) {
    led1state = val;
    led2state = 0;
    led3state = 0;
    led4state = 0;
    led5state = 0;
  } else {
    led1state = 0;
    led2state = 0;
    led3state = 0;
    led4state = 0;
    led5state = 0;
  }
  analogWrite(LED1, led1state);
  analogWrite(LED2, led2state);
  analogWrite(LED3, led3state);
  analogWrite(LED4, led4state);
  analogWrite(LED5, led5state);
}

int readBPM() {
  auto sample = sensor.readSample(1000);  // Read sensor data
  float current_value = sample.red;

  // Finger detection using raw sensor value
  if (current_value > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    // Reset filters and variables when the finger is removed
    differentiator.reset();
    averager.reset();
    low_pass_filter.reset();
    high_pass_filter.reset();
    finger_detected = false;
    finger_timestamp = millis();
    return -1;  // Return -1 if no finger is detected
  }

  if (finger_detected) {
    // Process signal with filters
    current_value = low_pass_filter.process(current_value);
    current_value = high_pass_filter.process(current_value);
    float current_diff = differentiator.process(current_value);

    if (!isnan(current_diff) && !isnan(last_diff)) {
      // Detect zero-crossing (heartbeat event)
      if (last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0) {
        crossed = false;
      }

      // Detect falling edge of the waveform
      if (crossed && current_diff < kEdgeThreshold) {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Calculate BPM
          int raw_bpm = 60000 / (crossed_time - last_heartbeat);
          bpm_out = raw_bpm;
          // // Debug
          // Serial.print("BPM raw:");
          // Serial.println(raw_bpm);

          // Apply averaging if enabled
          if (kEnableAveraging) {
            int avg_bpm = averager.process(raw_bpm);

            // Update bpm_out only if enough samples have been averaged
            if (averager.count() > kSampleThreshold) {
              bpm_out = avg_bpm;
              // Debug
              Serial.print("BPM avg:");
              Serial.println(raw_bpm);
            }
          }
        }

        // Reset for next detection
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }

  // Return bpm_out (default is 0 if no heartbeat detected yet)
  return bpm_out;  // Return -1 if BPM is invalid
  // delay(1000);
}
