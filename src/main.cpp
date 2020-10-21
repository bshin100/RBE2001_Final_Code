/**
 * Main controller class for the RBE 2001 Final Project
 * @author Brian Shin, with the guidance of Prof. Lewin & Prof. Miller
 */

#include <Arduino.h>
#include <Romi32U4.h>
#include "Chassis.h"
#include "BlueMotor.h"
#include "RangeFinder.h"
#include "servo32u4.h"
#include "RemoteConstants.h"
#include "IRdecoder.h"
#include "LineSensor.h"

Chassis chassis;
BlueMotor blueMotor;
Rangefinder rangeFinder;
Servo32U4 servo;
Romi32U4ButtonA pushButton;
Romi32U4ButtonB pushButtonB;
IRDecoder decoder;
LineSensor lineSensor;

// ----- CONFIG START ----- //
const bool TESTING = false;             // Enable testing sequence instead of autonomous sequence
const bool SKIP_TRAVERSE = true;        // Skip the ambitious traversal across the field to the other side
const bool ALUM_PLATE = true;           // Switch for aluminum-plate-specific code
bool AUTO_2 = false;
const float GEAR_RATIO_LIFTER = 30.8;   // Lifter gear ratio

                                        // LIFTER POSITIONS BELOW MUST BE MULTIPLIED BY GEAR RATIO
const float LIFTER_MAX = -142.0;        // Max height for the lifter, if zeroed when linkage touches nut
const float LIFTER_PLATFORM = -15.0;     // Lifter position for height of platform
const float LIFTER_25ROOF = -115.0;     // Lifter position for height of 25-degree roof
const float LIFTER_45ROOF = -81.5;      // Lifter position for height of 45-degree roof
const float DIST_PLATFORM = 13.75 - 2.0;      // Distance (in.) between front of robot and platform with wheels on intersection
const float DIST_ROOF = 13.45 - 3.0;          // DIstance (in.) between front of robot and roof inner panel with wheels on intersection
const int GRIPPER_CLOSED = 815;         // Servo position for closed gripper
const int GRIPPER_OPEN = 1700;          // Servo position for open gripper

/* Additional configuration parameters can be found in the header files of specific classes.*/
// ----- CONFIG END -----//

bool paused = false;
bool tweak = false;

enum States {
    SETUP_RAISE,
    CONFIRM_SETUP,
    GRIPPING_1,
    CONFIRM_1,
    DRIVE_REV_LOWER_1,
    TURN_LEFT_1,
    DRIVE_FWD_PLATFORM,
    CONFIRM_2, 
    RELEASE_1,
    CONFIRM_3,
    GRIPPING_2,
    DRIVE_REV_LIFT_1,
    TURN_RIGHT_1,
    DRIVE_FWD_ROOF,
    CONFIRM_DEPOSIT,
    RELEASE_2,

    IDLE,           // Universal Idle
    STOPPED         // Universal E-Stop
} state;

const char *stateNames[] = {"SETUP_RAISE", "CONFIRM_SETUP", "GRIPPING_1", "CONFIRM_1", "DRIVE_REV_LOWER_1", 
    "TURN_LEFT_1", "DRIVE_FWD_PLATFORM", "CONFIRM_2", "RELEASE_1", "CONFIRM_3", "GRIPPING_2", "DRIVE_REV_LIFT_1", 
    "TURN_RIGHT_1", "DRIVE_FWD_ROOF", "CONFIRM_DEPOSIT", "RELEASE_2", "IDLE", "STOPPED"};

void checkRemote() {
    int keyCode = decoder.getKeyCode();
    if (keyCode == remotePlayPause) { // E-Stop Feature
        if(paused) {
            delay(250); // Attempt to stop bouncing
        }
        paused = !paused;
        Serial.println(paused ? "Paused" : "Running");
    } else if (keyCode == remoteSetup) {
        tweak = !tweak;
        Serial.println(tweak ? "Manual adjustment mode" : "Normal mode");
    } else if (keyCode == remoteUp && tweak) {
        Serial.println("Up Button: Adjusting lifter up");
        blueMotor.setEffortWithoutDB(-20);
    } else if (keyCode == remoteDown && tweak) {
        Serial.println("Down Button: Adjusting lifter down");
        blueMotor.setEffortWithoutDB(50);
    } else if (keyCode == remoteEnterSave && tweak) {
        Serial.println("Enter Button: Stopped lifter adjustment");
        blueMotor.setEffort(0);
    } else if (keyCode == remoteLeft && tweak) {
        Serial.println("Left Button: Gripper Open");
        servo.Write(GRIPPER_OPEN);
    } else if (keyCode == remoteRight && tweak) {
        Serial.println("Left Button: Gripper Closed");
        servo.Write(GRIPPER_CLOSED);
    } else if (keyCode == remote7) {
        AUTO_2 = true;
        Serial.println("AUTO 2 ENABLED");
    }
}

// Test things
void testSequence() {
    
    //Serial.println(rangeFinder.getDistance());
    //chassis.drive(100);
    //if(rangeFinder.getDistance() <= DIST_ROOF) {
     //   chassis.drive(0);
    //}
    // Serial.print("Dist: ");
    // Serial.print(rangeFinder.getDistance());
    // Serial.print("\t Targ: ");
    // Serial.println(chassis.pullOnTarget(rangeFinder.getDistance()));
    // //chassis.startUltraDrive(DIST_ROOF, rangeFinder.getDistance());
    // chassis.loopUltraPID(rangeFinder.getDistance());
    // if(chassis.pullOnTarget(rangeFinder.getDistance())) {
    //     chassis.drive(0);
    //     Serial.println("Target reached");
    // }

    lineSensor.loop();
    //lineSensor.setLineFollowForward();
    //chassis.setEffortsBoolean(lineSensor.leftDrive, lineSensor.rightDrive);
    chassis.startLinePID(lineSensor.readSensor(true), lineSensor.readSensor(false));
}

/*
Autonomous state machine for removing and placing a panel on the 25-degree roof.
Start directly under the 25 deg roof, gripper open.
1. Close Gripper                            2. Wait for confirm, Back up X CM
3. Lower Arm to platform height (2in)       4. Go back to platform using line follow + dead reckon
5. Wait for confirm, Open Gripper           6. Wait for confirm, Close gripper
7. Lift arm to 25 deg height                8. Return to structure with line follow + dead + ultrasonic (last leg)
9. Wait for confirm, Open Gripper           10. Back up X CM
*/
const bool skip_ir = true;

void autoSequence1() {
    static States previousState = IDLE;
    if (state != previousState) {
        Serial.println(stateNames[state]);
        previousState = state;
    } 

    switch (state) {
        case SETUP_RAISE:
        // Raise the arm from its zeroed position then open gripper.
        blueMotor.startMoveTo((LIFTER_25ROOF+1) * GEAR_RATIO_LIFTER);
        blueMotor.loopController();
        if (blueMotor.pullOnTarget()) {
            Serial.println("Lifter arm movement complete");
            blueMotor.setEffort(0);
            servo.Write(GRIPPER_OPEN);
            state = CONFIRM_SETUP;
        }
        break;

        case CONFIRM_SETUP:
        // Wait for confirmation from user that robot is properly placed
        static int tempCount1 = 0;
        if(tempCount1 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount1++;
            if(skip_ir) {
                delay(10000);
                paused = false;
            }
        }

        if(!paused) state = GRIPPING_1;
        break;

        case GRIPPING_1:
        // Close gripper on existing plate for removal
        servo.Write(GRIPPER_CLOSED);
        state = CONFIRM_1;
        break;
        
        case CONFIRM_1:
        // Wait for confirmation from user that plate is securely gripped
        static int tempCount2 = 0;
        if(tempCount2 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount2++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) {
            //chassis.startUltraDrive(DIST_ROOF, rangeFinder.getDistance());
            chassis.drive(-75);
            state = DRIVE_REV_LOWER_1;
        }
        break;

        case DRIVE_REV_LOWER_1:
        // Ultrasonic and drive in reverse until intersection, also lower arm to platform height
        //chassis.loopUltraPID(rangeFinder.getDistance());
        Serial.print("Dist: ");
        Serial.println(rangeFinder.getDistance());
        
        if(ALUM_PLATE) {
            blueMotor.startMoveTo((LIFTER_PLATFORM-14.5) * GEAR_RATIO_LIFTER);
        } else {
            blueMotor.startMoveTo(LIFTER_PLATFORM * GEAR_RATIO_LIFTER);
        }
        blueMotor.loopController();
        if (blueMotor.pullOnTarget()) {
            Serial.println("Lifter arm movement complete");
            blueMotor.setEffort(0);
        }
        /*if(chassis.pullOnTarget(rangeFinder.getDistance())) {
            chassis.drive(0);
            Serial.println("Chassis target reached");
        }*/
        if(rangeFinder.getDistance() >= DIST_ROOF) {
            Serial.println("Chassis target reached");
            chassis.drive(0);
        }
        if(rangeFinder.getDistance() >= DIST_ROOF/*chassis.pullOnTarget(rangeFinder.getDistance())*/ && blueMotor.pullOnTarget()) {
            Serial.println("both complete");
            state = TURN_LEFT_1;
            chassis.startTurn(-87);
        }
        break;

        case TURN_LEFT_1:
        // Turn left 90 degrees to face platform
        if(chassis.turnComplete()) {
            Serial.println("Turn 90 left complete");
            chassis.drive(0);
            state = DRIVE_FWD_PLATFORM;
            delay(1000);
            chassis.setEfforts(72,75);
            //chassis.startUltraDrive(2.5, rangeFinder.getDistance());
        }
        break;

        case DRIVE_FWD_PLATFORM:
        // Drive to platform with linefollow/ultrasonic/both
        //chassis.loopUltraPID(rangeFinder.getDistance());
        if(rangeFinder.getDistance() <= 2.5) {
        //if(chassis.pullOnTarget(rangeFinder.getDistance())) {
            Serial.println("Chassis target reached");
            chassis.drive(0);
            state = CONFIRM_2;
        }
        break;

        case CONFIRM_2:
        // Wait for confirmation from user that plate is placed on platform
        static int tempCount3 = 0;
        if(tempCount3 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount3++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = RELEASE_1;
        break;

        case RELEASE_1:
        // Open gripper to place plate on platform
        servo.Write(GRIPPER_OPEN);
        state = CONFIRM_3;
        break;

        case CONFIRM_3:
        // Wait for confirmation from user that old plate is removed and new plate is on platform
        static int tempCount4 = 0;
        if(tempCount4 == 0) {
            Serial.println("Awaiting user confirmation");
            paused = true;
            tempCount4++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = GRIPPING_2;
        break;

        case GRIPPING_2:
        // Close gripper on new plate
        servo.Write(GRIPPER_CLOSED);
        delay(250);
        state = DRIVE_REV_LIFT_1;
        chassis.setEfforts(-75, -75);
        break;

        case DRIVE_REV_LIFT_1:
        // Line follow and drive in reverse until intersection, also raise arm to 25 deg roof height
        if(rangeFinder.getDistance() >= DIST_PLATFORM-0.3) {
            Serial.println("Chassis target reached.");
            chassis.drive(0);
        }
        blueMotor.startMoveTo(LIFTER_25ROOF * GEAR_RATIO_LIFTER);
        blueMotor.loopController();
        if (blueMotor.pullOnTarget()) {
            Serial.println("Lifter arm movement complete");
            blueMotor.setEffort(0);
            state = TURN_RIGHT_1;
            chassis.startTurn(84);
        }
        break;

        case TURN_RIGHT_1:
        // Turn right 90 degrees to face roof
        if(chassis.turnComplete()) {
            Serial.println("Turn 90 right complete");
            chassis.drive(0);
            state = DRIVE_FWD_ROOF;
            delay(500);
            chassis.setEfforts(65, 75);
        }
        break;

        case DRIVE_FWD_ROOF:
        // Drive to roof with linefollow/ultrasonic/both
        if(rangeFinder.getDistance() <= 4.14) {
            Serial.println("Chassis target reached.");
            chassis.drive(0);
            state = CONFIRM_DEPOSIT;
        }
        break;

        case CONFIRM_DEPOSIT:
        // Wait for confirmation from user that plate is alligned with roof
        static int tempCount5 = 0;
        if(tempCount5 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount5++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = RELEASE_2;
        break;

        case RELEASE_2:
        // Open gripper to place plate on 25 deg roof and back up to line again
        servo.Write(GRIPPER_OPEN);
        static int tempCount6 = 0;
        if(tempCount6 == 0) {
            delay(500);
            chassis.setEfforts(-75,-75);
            tempCount6++;
        }
        if(rangeFinder.getDistance() >= DIST_ROOF) {
            Serial.println("Chassis target reached.");
            chassis.drive(0);
            state = IDLE;
        }
        break;

        case IDLE:
        Serial.println("Autonomous sequence complete.");
        break;

        case STOPPED:
        paused = true;
        break;
    }
}

/*
Autonomous state machine for removing and placing a panel on the 45-degree roof.
Start directly under the 45 deg roof, gripper open.
*/
void autoSequence2() {
    static States previousState = IDLE;
    if (state != previousState) {
        Serial.println(stateNames[state]);
        previousState = state;
    } 

    switch (state) {
        case SETUP_RAISE:
        // Raise the arm from its zeroed position then open gripper.
        blueMotor.startMoveTo(LIFTER_45ROOF * GEAR_RATIO_LIFTER);
        blueMotor.loopController();
        if (blueMotor.pullOnTarget()) {
            Serial.println("Lifter arm movement complete");
            blueMotor.setEffort(0);
            servo.Write(GRIPPER_OPEN);
            state = CONFIRM_SETUP;
        }
        break;

        case CONFIRM_SETUP:
        // Wait for confirmation from user that robot is properly placed
        static int tempCount1 = 0;
        if(tempCount1 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount1++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = GRIPPING_1;
        break;

        case GRIPPING_1:
        // Close gripper on existing plate for removal
        servo.Write(GRIPPER_CLOSED);
        state = CONFIRM_1;
        break;
        
        case CONFIRM_1:
        // Wait for confirmation from user that plate is securely gripped
        static int tempCount2 = 0;
        if(tempCount2 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount2++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) {
            //chassis.startUltraDrive(DIST_ROOF, rangeFinder.getDistance());
            chassis.setEfforts(-75, -75);
            state = DRIVE_REV_LOWER_1;
        }
        break;

        case DRIVE_REV_LOWER_1:
        // Ultrasonic and drive in reverse until intersection, also lower arm to platform height
        //chassis.loopUltraPID(rangeFinder.getDistance());
        Serial.print("Dist: ");
        Serial.println(rangeFinder.getDistance());
        
        static bool delayFlag = false;
        if(rangeFinder.getDistance() >= DIST_ROOF+2.35) {
            Serial.println("Chassis target reached");
            chassis.drive(0);
            delayFlag = true;
        }

        if(delayFlag) blueMotor.startMoveTo(LIFTER_PLATFORM * GEAR_RATIO_LIFTER);
        if(delayFlag) blueMotor.loopController();
        if (delayFlag && blueMotor.pullOnTarget()) {
            Serial.println("Lifter arm movement complete");
            blueMotor.setEffort(0);
        }
        /*if(chassis.pullOnTarget(rangeFinder.getDistance())) {
            chassis.drive(0);
            Serial.println("Chassis target reached");
        }*/
        
        if(rangeFinder.getDistance() >= DIST_ROOF+2.35/*chassis.pullOnTarget(rangeFinder.getDistance())*/ && blueMotor.pullOnTarget()) {
            Serial.println("both complete");
            state = TURN_LEFT_1;
            chassis.startTurn(79);
        }
        break;

        case TURN_LEFT_1:                           // This is actually turn RIGHT lol. I'll change later...?
        // Turn right 90 degrees to face platform
        if(chassis.turnComplete()) {
            Serial.println("Turn 90 left complete");
            chassis.drive(0);
            state = DRIVE_FWD_PLATFORM;
            delay(1000);
            chassis.setEfforts(70,75);
            //chassis.startUltraDrive(2.5, rangeFinder.getDistance());
        }
        break;

        case DRIVE_FWD_PLATFORM:
        // Drive to platform with linefollow/ultrasonic/both
        //chassis.loopUltraPID(rangeFinder.getDistance());
        if(rangeFinder.getDistance() <= 2.25) {
        //if(chassis.pullOnTarget(rangeFinder.getDistance())) {
            Serial.println("Chassis target reached");
            chassis.drive(0);
            state = CONFIRM_2;
        }
        break;

        case CONFIRM_2:
        // Wait for confirmation from user that plate is placed on platform
        static int tempCount3 = 0;
        if(tempCount3 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount3++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = RELEASE_1;
        break;

        case RELEASE_1:
        // Open gripper to place plate on platform
        servo.Write(GRIPPER_OPEN);
        state = CONFIRM_3;
        break;

        case CONFIRM_3:
        // Wait for confirmation from user that old plate is removed and new plate is on platform
        static int tempCount4 = 0;
        if(tempCount4 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount4++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = GRIPPING_2;
        break;

        case GRIPPING_2:
        // Close gripper on new plate
        servo.Write(GRIPPER_CLOSED);
        delay(250);
        state = DRIVE_REV_LIFT_1;
        chassis.setEfforts(-68, -75);
        break;

        case DRIVE_REV_LIFT_1:
        // Line follow and drive in reverse until intersection, also raise arm to 45 deg roof height
        if(rangeFinder.getDistance() >= DIST_PLATFORM+6.5) {
            Serial.println("Chassis target reached.");
            chassis.drive(0);
        }
        blueMotor.startMoveTo(LIFTER_45ROOF * GEAR_RATIO_LIFTER);
        blueMotor.loopController();
        if (blueMotor.pullOnTarget()) {
            Serial.println("Lifter arm movement complete");
            blueMotor.setEffort(0);
            state = TURN_RIGHT_1;
            chassis.startTurn(-85);
        }
        break;

        case TURN_RIGHT_1:                          // This is actually turn LEFT lmfao, same thing as above.
        // Turn left 90 degrees to face roof
        if(chassis.turnComplete()) {
            Serial.println("Turn 90 left complete");
            chassis.drive(0);
            state = DRIVE_FWD_ROOF;
            delay(500);
            chassis.setEfforts(67, 75);
        }
        break;

        case DRIVE_FWD_ROOF:
        // Drive to roof with linefollow/ultrasonic/both
        if(rangeFinder.getDistance() <= 4.84) {
            Serial.println("Chassis target reached.");
            chassis.drive(0);
            state = CONFIRM_DEPOSIT;
        }
        break;

        case CONFIRM_DEPOSIT:
        // Wait for confirmation from user that plate is alligned with roof
        static int tempCount5 = 0;
        if(tempCount5 == 0) {
            Serial.print("Awaiting user confirmation");
            paused = true;
            tempCount5++;
            if(skip_ir) {
                delay(5000);
                paused = false;
            }
        }

        if(!paused) state = RELEASE_2;
        break;

        case RELEASE_2:
        // Open gripper to place plate on 25 deg roof and back up to line again
        servo.Write(GRIPPER_OPEN);
        static int tempCount6 = 0;
        if(tempCount6 == 0) {
            delay(500);
            chassis.setEfforts(-75,-75);
            tempCount6++;
        }
        if(rangeFinder.getDistance() >= DIST_ROOF) {
            Serial.println("Chassis target reached.");
            chassis.drive(0);
            state = IDLE;
        }
        break;

        case IDLE:
        Serial.println("Autonomous sequence complete.");
        break;

        case STOPPED:
        paused = true;
        break;
    }
}

/**
 * Set up the various subsystems.
 */
void setup() {
    Serial.begin(9600);
    Serial.println("Beginning Program");

    decoder.init();

    blueMotor.setup();
    blueMotor.setEffort(0);
    blueMotor.reset();

    servo.Init();
    servo.Attach();
    servo.SetMinMaxUS(GRIPPER_CLOSED, GRIPPER_OPEN);
    pinMode(18, INPUT);
    servo.Write(GRIPPER_CLOSED); // Start the gripper closed.

    rangeFinder.setup();
    lineSensor.setup();
    chassis.setup();

    state = SETUP_RAISE;

    while(!pushButton.isPressed()) delay(10); // Wait for button to start
    if(TESTING) chassis.startUltraDrive(DIST_ROOF, rangeFinder.getDistance());
}

/** 
 * Core loop of the controller, runs remote check and then the main sequencing/functions.
 * Note that the runtime/latency of any functions here will inhibit the latency of the remote functionality,
 * namely the E-Stop feature (which isn't an E-Stop if its not instantaneous).
 */
void loop() {
  
  checkRemote();
  rangeFinder.loop();
  
  if(paused) {
      //Serial.print("Code paused. Last state: ");
      //Serial.println(stateNames[state]);
      
      chassis.drive(0);     // Turn off all motors
      blueMotor.setEffort(0);
  } else {
      // Run main sequencing
      if(!TESTING && !AUTO_2) autoSequence1(); else if(!TESTING && AUTO_2) autoSequence2(); else testSequence();
      //autoSequence1();
  }
}