/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

//CONTROLLER 0 is operator
//CONTROLLER 1 is driver

namespace OIConstants {
    constexpr static int GAMEPAD_BASE_LOCATION = 1;
    constexpr static int GAMEPAD_OPERATOR_LOCATION = 0;

    // Default Gateway 10.68.0.1
    // Rio 10.68.0.2
    // Limelight 10.68.0.11:5801
    // mask 24
/**
 * Limelight tuning values
 * Input TAB
 *  Exposure: 2
 *  Black level offset: 5
 *  Red Balance: 1500
 *  Blue Balance: 1920
 * Threshholding
 *  Hue: 0 - 179
 *  Saturation: 118 - 255
 *  Value: 152 - 255
 * Contour Filtering
 *  Area: 0.0534 - 100.000
 *  Fullness: 0.2 - 100.0
 *  W/H Ratio: 0.5120 - 0.8201
 *  Direction Filter: none
 *  Target Grouping: Single Target
 */
}

namespace DriveConstants {
    constexpr static int CAN_ID_LEFT_A = 1; //changed to ID=11 for testing
    constexpr static int CAN_ID_LEFT_B = 3;
    constexpr static int CAN_ID_RIGHT_A = 2;
    constexpr static int CAN_ID_RIGHT_B = 4;

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.1;
    constexpr static double kArcTurnMultipler = 0.5;
    constexpr static double kNoBoost = 0.5;
    constexpr static double kBoost = 1;
    constexpr static double kDriveMultiplierX = 0.60;
    constexpr static double kDriveMultiplierY = 1;
    constexpr static double limeLightKP = 0.005;
}

namespace ShooterConstants {
    constexpr static int CAN_ID_FLYWHEEL_A = 11; //changed to ID=1 for testing
    constexpr static int CAN_ID_FLYWHEEL_B = 12;
    constexpr static int CAN_ID_TURRET = 6;
    constexpr static int SOLENOID_ID_SHOOTER = 1;

    // Deadband for the left joystick x axis
    constexpr static double kDeadband = 0.05;

    constexpr static int dpadUp = 0;
    constexpr static int dpadRight = 90;
    constexpr static int dpadDown = 180;
    constexpr static int dpadLeft = 270;

    // Turret input P control value
    constexpr static double turretKP = 1/16.0;
    // Turret power deadband. Minimum power required to move the turret
    constexpr static double pDeadband = 1;
    // Turret limelight P control value
    constexpr static double limelightTurnKp = 0.2;
    constexpr static double limelightDistKp = 0.2;


    // Fender shot successful: throat 50%, shooter 75%
    // initiation: shooter 65%
    // trench: 70%
    constexpr static double fenderPower = 0.2;
    constexpr static double initiationPower = 0.2;
    constexpr static double trenchPower = 0.2;
    constexpr static double defaultManualPower = 0.6;

    //turret encoder
    //encoder is on outputshaft of neo
    //5 rotations of neo ~= 20 degree of rotation
    //facing front of robot, counter clockwise is positive, clockwise is negative

    constexpr static double homePosition = 0;

    // Encoder ticks off of center
    // 192 (gear ration) * angle ratio (ex. 1/2 for 180 deg)
    constexpr static double limitLeft = homePosition + 96; // 48;
    constexpr static double limitRight = homePosition - 96; // 96;
}

namespace SpindexerConstants {
    constexpr static int CAN_ID = 10;
    constexpr static int CAN_ID_THROAT = 5;
    constexpr static int CAN_ID_THROAT_FOLLOW = 7;

    constexpr static double left_trigger_deadband = 0.05;
    constexpr static double default_throat_spd = 0.6;
    constexpr static double default_drum_spd = 0.3;
    constexpr static double high_spd_drum = 0.8;
}

namespace LiftConstants {
    constexpr static int MOTOR_CAN_ID = 9;
    constexpr static int MOTOR_FOLLOW_CAN_ID = 12;

    constexpr static int LIMIT_DIO = 0;
    constexpr static int POT_ANOLOG_PORT = 0;

    constexpr static double POT_RANGE_SCALE = 1.0;
    constexpr static double POT_RANGE_OFFSET = 0.0;
}

namespace IntakeConstants {
    constexpr static int DEFAULT_ROLLER_SPD = 0.6;

    constexpr static int MOTOR_CAN_ID = 13; //change to correct value

    constexpr static int SOLENOID_FORWARD_PCM_CAN_ID = 0;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

#endif
