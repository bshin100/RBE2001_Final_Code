#include <Arduino.h>
#include "LineSensor.h"

void LineSensor::setup() {
    pinMode(leftSensorPin, INPUT);  // Left sensor
    pinMode(rightSensorPin, INPUT);  // Right sensor (blue motor side)
}

void LineSensor::loop() {
    senseLeftOut = analogRead(leftSensorPin);
    senseRightOut = analogRead(rightSensorPin);

    // Serial.print("Sensor 1: ");
    // Serial.print(senseLeftOut);
    // Serial.print("\tSensor 2: ");
    // Serial.println(senseRightOut);
}

/**
 * Read the sensor values and return them
 * @param left A boolean value indicating left sensor or right sensor
 * @return Integer reading from sensor, 0 - 1023.
 */
int LineSensor::readSensor(bool left) {
    senseLeftOut = analogRead(leftSensorPin);
    senseRightOut = analogRead(rightSensorPin);

    return (left) ? senseLeftOut : senseRightOut;
}

/**
 * Follow a line forward
 */
void LineSensor::setLineFollowForward() {
    int left_sensor_state = readSensor(true);
    int right_sensor_state = readSensor(false);

  if(right_sensor_state < 500 && left_sensor_state > 500){
    Serial.println("turning right");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");

    leftDrive = true;
    rightDrive = false;
    //delay(10);
  }
  if(right_sensor_state > 500 && left_sensor_state < 500){
    Serial.println("turning left");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");   

    leftDrive = false;
    rightDrive = true;
    //delay(10);
  }

  if(right_sensor_state < 500 && left_sensor_state < 500){
    Serial.println("going forward");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");

    leftDrive = true;
    rightDrive = true;
    //delay(10);
  }

  if(right_sensor_state > 500 && left_sensor_state > 500){ 
    Serial.println("stop");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");

    leftDrive = false;
    rightDrive = false;
    //delay(10);
  }
}

void LineSensor::setLineFollowBackward() {
    int left_sensor_state = readSensor(true);
    int right_sensor_state = readSensor(false);

  if(right_sensor_state > 500 && left_sensor_state < 500){
    Serial.println("turning left");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");

    leftDrive = false;
    rightDrive = true;
    //delay(100);
  }
  if(right_sensor_state < 500 && left_sensor_state > 500){
    Serial.println("turning right");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");    

    leftDrive = true;
    rightDrive = false;
  }

  if(right_sensor_state > 500 && left_sensor_state > 500){
    Serial.println("going backward");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");

    leftDrive = true; // Both need to be in reverse
    rightDrive = true;
  }

  if(right_sensor_state < 500 && left_sensor_state < 500){ 
    Serial.println("stop");
    Serial.println(right_sensor_state);
    Serial.println(left_sensor_state);
    Serial.println("------------------");    

    leftDrive = false;
    rightDrive = false;
  }
}