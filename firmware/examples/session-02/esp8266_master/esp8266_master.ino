// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Mood Credit Card contributors

// Master Code
// This code is to be uploaded onto the ESP8266
// The code stores the input from the serial monitor as a string
// It sends the string via I2C

// Include the required Wire library for I2C
#include <Wire.h>

#define BUFFER_SIZE 32 // Maximum I2C buffer size
char inputText[BUFFER_SIZE]; // Buffer for user input

void setup() {
  // Start the I2C Bus as Master
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Type a message and press Enter to send:");
}

void loop() {
  // Check if data is available to read from the Serial Monitor
  if (Serial.available() > 0) {
    // Read characters from the Serial Monitor until a newline ('\n') is encountered
    // or until the buffer is full (BUFFER_SIZE - 1)
    size_t len = Serial.readBytesUntil('\n', inputText, BUFFER_SIZE - 1);

    // Add a null-terminator at the end of the input to mark it as a valid C-string
    inputText[len] = '\0';

    // Begin communication with the I2C slave device at address 9
    Wire.beginTransmission(9);

    // Send the input text string to the slave device over I2C
    Wire.write(inputText);

    // End the I2C transmission
    Wire.endTransmission();

    // Print a confirmation message to the Serial Monitor showing what was sent
    Serial.println("Sent: " + String(inputText));

    // Prompt the user to type another message
    Serial.println("Type another message:");
  }
}
