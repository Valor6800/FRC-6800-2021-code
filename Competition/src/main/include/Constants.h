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
}

namespace DriveConstants {
    constexpr static int CAN_ID_LEFT_A = 1;
    constexpr static int CAN_ID_LEFT_B = 3;
    constexpr static int CAN_ID_RIGHT_A = 2;
    constexpr static int CAN_ID_RIGHT_B = 4;

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.1;
    constexpr static double kArcTurnMultipler = 0.5;
    constexpr static double kNoBoost = 0.5;
    constexpr static double kBoost = 1;
    constexpr static double kDriveMultiplierX = 0.75;
    constexpr static double kDriveMultiplierY = 1;
}

namespace ShooterConstants {
    constexpr static int CAN_ID_FLYWHEEL_A = 11;
    constexpr static int CAN_ID_FLYWHEEL_B = 12;
    constexpr static int CAN_ID_TURRET = 6;
    constexpr static int SOLENOID_ID_SHOOTER = 1;

    // Deadband for the X axis gamepad thumbpad
    constexpr static double kDeadband = 0.05;

    constexpr static int dpadUp = 0;
    constexpr static int dpadRight = 90;
    constexpr static int dpadDown = 180;
    constexpr static int dpadLeft = 270;

    // Turret input P control value
    constexpr static double turretKP = 0.2;
    // Turret power deadband. Minimum power required to move the turret
    constexpr static double pDeadband = 0;
    // Turret limelight P control value
    constexpr static double limelightTurnKp = 0.2;
    constexpr static double limelightDistKp = 0.2;

    constexpr static double fenderPower = 0.6;
    constexpr static double initiationPower = 0.6;
    constexpr static double trenchPower = 0.6;
}

namespace SpindexerConstants {
    constexpr static int CAN_ID = 9;
    constexpr static int CAN_ID_THROAT = 5;
    constexpr static int CAN_ID_THROAT_FOLLOW = 7;

    constexpr static double left_trigger_deadband = 0.05;
}

namespace LiftConstants {
    constexpr static int MOTOR_CAN_ID = 10;
    constexpr static int MOTOR_FOLLOW_CAN_ID = 12;

    constexpr static int LIMIT_DIO = 0;
    constexpr static int POT_ANOLOG_PORT = 0;

    constexpr static double POT_RANGE_SCALE = 1.0;
    constexpr static double POT_RANGE_OFFSET = 0.0;
}

namespace IntakeConstants {
    constexpr static int MOTOR_CAN_ID = 13; //change to correct value

    constexpr static int SOLENOID_FORWARD_PCM_CAN_ID = 3;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

#endif
