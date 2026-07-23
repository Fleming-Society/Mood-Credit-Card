// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Mood Credit Card contributors

// Slave Code
// This code is to be uploaded onto the Arduino Uno
// The code receives any data (string) that was sent
// The code prints the string into the serial monitor

// Include the required Wire library for I2C
#include <Wire.h>

#define BUFFER_SIZE 32 // Maximum I2C buffer size

char receivedText[BUFFER_SIZE]; // Buffer to store received data
int textIndex = 0;

void setup() {
  // Start the I2C Bus as Slave on address 9
  Serial.begin(9600);
  Wire.begin(9);
  // Attach a function to trigger when something is received
  Wire.onReceive(receiveEvent);
}

void receiveEvent(int bytes) {
  // Reset the index to 0 to prepare for storing a new message
  textIndex = 0;

  // Continue reading data from the I2C buffer as long as there are bytes available
  while (Wire.available()) {
    char c = Wire.read(); // Read one character from the I2C buffer

    // Check if there is space in the buffer to store the character
    // Leave room for the null-terminator to mark the end of the string
    if (textIndex < BUFFER_SIZE - 1) {
      receivedText[textIndex++] = c; // Store the character and increment the index
    }
  }

  // Add a null-terminator at the end of the array to convert it into a proper C-string
  receivedText[textIndex] = '\0';
}

void loop() {
  // Check if a message has been received (textIndex > 0 means data is in the buffer)
  if (textIndex > 0) {
    // Print the received message to the Serial Monitor
    Serial.println("Received: " + String(receivedText));

    // Reset the index to 0 to indicate the message has been processed
    // and the buffer is ready for the next message
    textIndex = 0;
  }
}
