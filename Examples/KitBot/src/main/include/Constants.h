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
}

#endif
