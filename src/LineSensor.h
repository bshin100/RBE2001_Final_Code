#include <Arduino.h>

#pragma once

class LineSensor {
    public:
        void setup();
        void loop();
        int readSensor(bool left);
        void setLineFollowForward();
        void setLineFollowBackward();

        bool leftDrive = false;
        bool rightDrive = false;

    private:
        const int leftSensorPin = 20;
        const int rightSensorPin = 21;
        const int TABLE_THRESHOLD = 60; // Sensor reading for the reflectance of the lab table 
        const int LINE_THRESHOLD = 800; // Sensor reading for the reflectance of the line (black tape)

        int senseLeftOut;
        int senseRightOut;
};