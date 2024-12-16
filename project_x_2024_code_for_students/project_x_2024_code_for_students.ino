//------------------------------------------------------------------------------------------------//
// Here are the codes for the Project X. the heartrate readout function has been packaged
// into a function, called "readBPM()". This function returns a valid value if the sensor
// have detected the heartrate. it returns -1 if a faulty signal has appeared

// You might need to install the MAX3010x Sensor Library on the lab computer to make sure 
// everything is functioning well. To install the library, go to Tools/Manage Libraries
// and search for "MAX3010x Sensor Library", then install it. If the program passes the 
// compile, that means the library have been installed and you are good to go.

// Please write your codes in the ----- void main() { <HERE> } -----, there are few lines 
// of instructions that may or may not help with your coding process in the main loop.

// Good luck, and have fun coding!
// Junzhe Chen, 09/12/2024

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

// Timer
long currentMillis, prevMillis = 0;

// Filter Instances
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);

//------------------------------------------------------------------------------------------------//
//----- Define your variables here -----//
int bpm_out = 0;


//----- Defined variables ends here -----//

//----- Setup Function (runs once) -----//
void setup() {
  Serial.begin(115200);  //baud rate

  //Sensor initialisation
  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    while (1)
      ;
  }
}

//------------------------------------------------------------------------------------------------//
//----- Main Function (repeats forever), write your codes below -----//
void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis > 1000) {
    // Function inside this loop get called once per 1000 ms
    // You can put your BPM update, LED brightness update or anything else here





    prevMillis = currentMillis;
  }

  //----- Define your LEDs here -----//
  // analogWrite(<#LED>, <brightness>) can be used to give LED variable brightness
  // digitalWRite(#LED>, <HIGH/LOW>) will output either fully bright / fully dark

  // Try using if-else statement, analog/digitalWrite to control the LED
  // regarding to your detected heart rate!




  
}

//------------------------------------------------------------------------------------------------//
//----- pre-defined function, DO NOT CHANGE -----//
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
        }

        // Reset for next detection
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
  return bpm_out;
}
