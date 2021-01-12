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

namespace OIConstants {
    constexpr static int GAMEPAD_BASE_LOCATION = 1;
    constexpr static int GAMEPAD_OPERATOR_LOCATION = 0;
}

namespace DriveConstants {
    constexpr static int CAN_ID_LEFT_A = 1;
    constexpr static int CAN_ID_LEFT_B = 2;
    constexpr static int VICTOR_ID_RIGHT_A = 2;
    constexpr static int VICTOR_ID_RIGHT_B = 3;

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.05;
    constexpr static double kArcTurnMultipler = 0.5;
    constexpr static double kNoBoost = 0.5;
    constexpr static double kBoost = 1;
    constexpr static double kDriveMultiplierX = 0.5;
    constexpr static double kDriveMultiplierY = 1;
}

namespace SpindexerConstants {
    constexpr static int CAN_ID = 4;
}

namespace LiftConstants {
    constexpr static int LEFT_CAN_ID = 13;
    constexpr static int RIGHT_CAN_ID = 3;

    constexpr static int LIMIT_DIO = 0;
    constexpr static int POT_ANOLOG_PORT = 0;

    constexpr static double POT_RANGE_SCALE = 1.0;
    constexpr static double POT_RANGE_OFFSET = 0.0;
}

namespace IntakeConstants {
    constexpr static int MOTOR_CAN_ID = 2;

    constexpr static int COMPRESSOR_PCM_ID = 0;
    constexpr static int SOLENOID_FORWARD_PCM_CAN_ID = 3;
    constexpr static int SOLENOID_REVERSE_PCM_CAN_ID = 4;
}

#endif