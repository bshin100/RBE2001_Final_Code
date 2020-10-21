#include <Arduino.h>
#include "Chassis.h" 

/**
 * Constructor for class Chassis
 */
Chassis::Chassis() {
    chassis_PID = PIDController(5.0, 0.1, 0.05);
    line_PID = PIDController(0.1, 0.00, 0.01);
}

/**
 * Drive straight with a given effort.
 * @param effort Desired effort, 0 - 300.
 */
void Chassis::drive(float effort) {
    motors.setEfforts(effort, effort);
}

void Chassis::setEfforts(float effortLeft, float effortRight) {
    motors.setEfforts(effortLeft, effortRight);
}

void Chassis::setEffortsBoolean(bool effortLeft, bool effortRight) {
    if(effortLeft && !effortRight) {
        motors.setEfforts(SPEED_VAL, 0);
    } else if(!effortLeft && effortRight) {
        motors.setEfforts(0, SPEED_VAL);
    } else if(effortLeft, effortRight) {
        motors.setEfforts(SPEED_VAL, SPEED_VAL);
    } else if(!effortLeft && !effortRight) {
        motors.setEfforts(0, 0);     
    }
}

/** 
 * Drive straight for a given distance, this is blocking and has its own loop, so this should be run only once (not in another loop)
 * @param inches Desired distance to drive.
 */ 
void Chassis::driveDistance(float inches) {
    encoders.getCountsAndResetLeft(); // Reset encoders
    encoders.getCountsAndResetRight();

    float countSetpoint = inches * (Chassis::CPR / (Chassis::wheelDiameter * (PI)));

    while((abs(encoders.getCountsLeft()) < countSetpoint) && (abs(encoders.getCountsRight()) < countSetpoint)) {
        motors.setEfforts(SPEED_VAL, SPEED_VAL);
    }
    motors.setEfforts(0, 0); // Stop after completion
} 

/**
 * Non-blocking method to start driving a set distance with ultrasonic sensor input
 * @param inches Desired distance to drive
 * @param ultraInput Ultrasonic sensor reading, in inches
 */
void Chassis::startUltraDrive(float inches, float ultraInput) {
    chassis_PID.setSetpoint(inches);
    float eff = chassis_PID.calculateEffort(ultraInput);

    if(eff > 0 && abs(eff) < SPEED_VAL) { // + effort; below max
        drive(eff);
    } else if(eff > 0 && abs(eff) > SPEED_VAL) { // + effort; above max
        drive(SPEED_VAL);
    } else if(eff < 0 && abs(eff) < SPEED_VAL) { // - effort; below max
        drive(eff);
    } else if(eff < 0 && abs(eff) > SPEED_VAL) { // - effort; above max
        drive(-SPEED_VAL);
    }
    delay(10); // 100Hz controller
}

/**
 * Accompanying function to `startUltraDrive` that runs the PID algorithm
 */
void Chassis::loopUltraPID(float ultraInput) {
    if(!chassis_PID.onTarget(ultraInput)) {
        float eff = chassis_PID.calculateEffort(ultraInput);
        Serial.println(eff);
        
        if(eff > 0 && abs(eff) < SPEED_VAL) { // + effort; below max
            drive(eff);
        } else if(eff > 0 && abs(eff) > SPEED_VAL) { // + effort; above max
            drive(SPEED_VAL);
        } else if(eff < 0 && abs(eff) < SPEED_VAL) { // - effort; below max
            drive(eff);
        } else if(eff < 0 && abs(eff) > SPEED_VAL) { // - effort; above max
            drive(-SPEED_VAL);
        }

        delay(10); // 100Hz controller
    } else {
        drive(0);
    }
}

/**
 * Pull the PID onTarget status from the local PIDController instance
 * @param ultraInput Ultrasonic sensor reading, in inches
 * @return True if the chassis is on target
 */
bool Chassis::pullOnTarget(float ultraInput) {
    return chassis_PID.onTarget(ultraInput);
}

/**
 * Follow a black line with PID control to the chassis 
 */
void Chassis::startLinePID(float lineLeftInput, float lineRightInput) {
    float leftDiff = lineLeftInput - lineRightInput;
    float rightDiff = lineLeftInput + lineRightInput;
    //line_PID.setSetpoint(1); // FIXME: find a target threshold
    //float eff = line_PID.calculateEffort();

    setEfforts(leftDiff * line_PID.getGainValue(1), rightDiff * line_PID.getGainValue(1));

    if(lineLeftInput < 800 && lineRightInput < 800) { // Detect an intersection and stop
        drive(0);
    }
}

/** 
 * Turn in place by a given angle, this is blocking and has its own loop, so this should be run only once.
 * Positive angle will dictate clockwise rotation (right turn) and vise versa.
 * @param degrees Desired angle to turn.
 */ 
void Chassis::turnAngle(float degrees) {     
    encoders.getCountsAndResetLeft(); // Reset encoders
    encoders.getCountsAndResetRight();

    float countSetpoint = ( ((degrees/360) * Chassis::wheelTrack) * (Chassis::CPR / Chassis::wheelDiameter) );

    while((abs(encoders.getCountsLeft()) < countSetpoint) && (abs(encoders.getCountsRight()) < countSetpoint)) {
        if(degrees > 0) motors.setEfforts(SPEED_VAL, -SPEED_VAL); else motors.setEfforts(-SPEED_VAL, SPEED_VAL);
    }
    motors.setEfforts(0, 0); // Stop after completion
} 

/**
 * Start a turn going to "degrees". This is non-blocking and sets the motor
 * speeds to a value for making the turn. Must be called once. Call turnComplete() to know
 * if the turn has reached the "degrees" goal.
 * @param degrees Desired angle to turn - positive is clockwise
 */
int finalTurnCount;
bool clockwise = true;
void Chassis::startTurn(float degrees) {
    finalTurnCount = abs(degrees) * CPR * wheelTrack / (360.0 * wheelDiameter);
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    if(degrees > 0) {
        clockwise = true;
        motors.setEfforts(SPEED_VAL, -SPEED_VAL); 
    } else { 
        clockwise = false;
        motors.setEfforts(-SPEED_VAL, SPEED_VAL);
    }
}

/**
 * Accompanying method to `startTurn()` to be called in a loop after starting the turn.
 * @return true if a turn started with startTurn() has completed
 */
bool Chassis::turnComplete() {
    if (clockwise) motors.setEfforts(SPEED_VAL, -SPEED_VAL); else motors.setEfforts(-SPEED_VAL, SPEED_VAL);
    bool retVal = (abs(encoders.getCountsLeft()) >= finalTurnCount) && (abs(encoders.getCountsRight()) >= finalTurnCount);
    return retVal;
}

void Chassis::setup() {
    chassis_PID.setTolerance(tolerance);
}