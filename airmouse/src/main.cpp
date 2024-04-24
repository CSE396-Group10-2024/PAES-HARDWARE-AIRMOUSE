#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <BleMouse.h>

uint8_t data[6], datas;
int16_t gyroX, gyroZ;
 
int Sensitivity = 600;
int delayi = 10;

BleMouse bleMouse;

#define LEFT_BUTTON 15 // GPIO15 pin connected to button
#define RIGHT_BUTTON 5 // GPIO5 pin connected to button
// #define RED_LED 12 // GPIO25 pin connected to LED
// #define GREEN_LED 25 // GPIO12 pin connected to LED

// Variables will change:
bool leftPressFlag = false;
bool rightPressFlag = false;
bool leftClickFlag = false;
unsigned long pressStartTime = 0;
const unsigned long clickThreshold = 200; // Adjust this threshold as needed (in milliseconds)

uint32_t timer;
uint8_t i2cData[14];
 
const uint8_t IMUAddress = 0x68;
const uint16_t I2C_TIMEOUT = 1000;
 
uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  return Wire.endTransmission(sendStop); // Returns 0 on success
}

uint8_t i2cWrite2(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
 
uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  if(Wire.endTransmission(false))
    return 1;
  Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true);
  for(uint8_t i = 0; i < nbytes; i++) {
    if(Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if(Wire.available())
        data[i] = Wire.read();
      else
        return 2;
    }
  }
  return 0;
}

void setup() {
  Wire.begin();

  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[3] = 0x00;

  while(i2cWrite(0x19, i2cData, 4, false));
  while(i2cWrite2(0x6B, 0x01, true));
  while(i2cRead(0x75, i2cData, 1));
  delay(100);
  while(i2cRead(0x3B,i2cData,6));
 
  timer = micros();
  Serial.begin(115200);
  bleMouse.begin();
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  // pinMode(RED_LED, OUTPUT);
  // pinMode(GREEN_LED, OUTPUT);

  // digitalWrite(RED_LED, LOW);
  // digitalWrite(GREEN_LED, LOW);

  delay(100);
}

void loop() {
  while(i2cRead(0x3B,i2cData,14));
 
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
 
  gyroX = gyroX / Sensitivity / 1.1  * 1;
  gyroZ = gyroZ / Sensitivity  * 1;

  if(bleMouse.isConnected()){
    // digitalWrite(GREEN_LED, HIGH);
    // Serial.print(gyroX);
    // Serial.print("   ");
    // Serial.print(gyroZ);
    // Serial.print("\r\n");
    bleMouse.move(gyroZ, gyroX);

    // Check button state to trigger left click
    // if (lastState == LOW && currentState == HIGH) { // Change condition based on your button wiring
    //   bleMouse.click(MOUSE_LEFT);
    // }

    // Check for button press (transition from LOW to HIGH)
    // if (lastState == LOW && currentState == HIGH) {
    //   // Button was just pressed
    //   bleMouse.press(MOUSE_LEFT); // Hold down the left mouse button
    //   buttonHeld = true; // Set button hold flag
    // }

    // // Check for button release (transition from HIGH to LOW)
    // if (lastState == HIGH && currentState == LOW) {
    //   // Button was just released
    //   if (buttonHeld) {
    //     // Release the left mouse button only if it was previously held down
    //     bleMouse.release(MOUSE_LEFT);
    //     buttonHeld = false; // Reset button hold flag
    //   }
    // }

    // save the last state
    // lastState = currentState;

    // this logic is flawed, it will not work as expected

    // if ((!digitalRead(LEFT_BUTTON)) && (!leftPressFlag)) {
    //   // implement click without holding
    //   // pressStartTime = millis(); // Record the start time of the press
    //   // if ((millis() - pressStartTime) >= clickThreshold) {
    //   //   leftPressFlag = true;
    //   //   bleMouse.press(MOUSE_LEFT);
    //   // } else {
    //   //   bleMouse.click(MOUSE_LEFT);
    //   // }

    //     leftPressFlag = true;
    //     bleMouse.press(MOUSE_LEFT);
    // } else if ((digitalRead(LEFT_BUTTON)) && (leftPressFlag)) {
    //   leftPressFlag = false;
    //   bleMouse.release(MOUSE_LEFT);
    // }

    // if (!digitalRead(RIGHT_BUTTON)) {
    //   bleMouse.click(MOUSE_RIGHT);
    // }

    // Check for left button click without holding
  if (digitalRead(LEFT_BUTTON) == LOW && !leftPressFlag) {
    // Record the start time of the press
    pressStartTime = millis();
    
    // Check if the button is released quickly (click)
    if ((millis() - pressStartTime) < clickThreshold) {
      leftPressFlag = true;
      bleMouse.press(MOUSE_LEFT);
    }
  } else if (digitalRead(LEFT_BUTTON) == HIGH && leftPressFlag) {
    // Release the left mouse button
    leftPressFlag = false;
    bleMouse.release(MOUSE_LEFT);
  }

  // Check for right button click
  // Check for right button click without holding
  if (digitalRead(RIGHT_BUTTON) == LOW && !rightPressFlag) {
    // Record the start time of the press
    pressStartTime = millis();
    
    // Check if the button is released quickly (click)
    if ((millis() - pressStartTime) < clickThreshold) {
      rightPressFlag = true;
      bleMouse.press(MOUSE_RIGHT);
    }
  } else if (digitalRead(RIGHT_BUTTON) == HIGH && rightPressFlag) {
    // Release the right mouse button
    rightPressFlag = false;
    bleMouse.release(MOUSE_RIGHT);
  }
    // digitalWrite(RED_LED, HIGH);
  }
   delay(delayi);
}