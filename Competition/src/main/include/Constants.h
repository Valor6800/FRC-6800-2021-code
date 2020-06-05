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

namespace OIConstants {
constexpr int kDriverControllerPort = 1;
}  // namespace OIConstants

namespace DriveConstants {

    // 1/2/3, 4/5/6 from pdp increasing towards intake
    constexpr static int CAN_ID_LEFT_LEAD = 1;
    constexpr static int CAN_ID_LEFT_FOLLOW_A = 2;
    constexpr static int CAN_ID_LEFT_FOLLOW_B = 3;
    constexpr static int CAN_ID_RIGHT_LEAD = 4;
    constexpr static int CAN_ID_RIGHT_FOLLOW_A = 5;
    constexpr static int CAN_ID_RIGHT_FOLLOW_B = 6;

    constexpr auto kTrackwidth = 0.68_m;

    constexpr double kWheelDiameterInches = 6;
    constexpr double kGearRatio = 8.8;

    constexpr static double kP = -0.01;
    constexpr static double kI = 0;
    constexpr static double kD = 0;
    constexpr static double kFF = 0.00017543859;
    constexpr static double kIz = 0;
    constexpr static double MAX_RPM = 5700;
    constexpr static double kMinOutput = -1.0;
    constexpr static double kMaxOutput = 1.0;
    constexpr static double kMinVel = 0;
    constexpr static double kMaxVel = 1000;
    constexpr static double kMaxAccel = 1500;
    constexpr static double kAllError = 0;
    constexpr static double kDeadband = 0.05;
    // changed x and y deadband to .05 for testing
    constexpr static double kDeadbandY = 0.05;
    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDriveMultiplierX = 0.5;
    constexpr static double kDriveMultiplierY = 1;
    constexpr static double kDriveOffset = 1;

}

namespace RamseteConstants {

    constexpr auto kMaxSpeed = 4.4_mps; // 4_mps
    constexpr auto kMaxAcceleration = 4_mps_sq; //4_mps_sq;

    constexpr double kRamseteB = 2;
    constexpr double kRamseteZeta = 0.7;

    // Convert rotations to meters using gear ratio and wheel diameter (converted to meters)
    constexpr double kPositionConversionFactor = DriveConstants::kWheelDiameterInches * 0.0254 / DriveConstants::kGearRatio * M_PI;
    // Convert rotations per minute to meters per second 
    constexpr double kVelocityConversionFactor = kPositionConversionFactor / 60;

    constexpr bool kGyroReversed = true;

    constexpr auto kS = 0.146_V;
    constexpr auto kV = 2.25 * 1_V * 1_s / 1_m;
    constexpr auto kA = 0.315 * 1_V * 1_s * 1_s / 1_m;

    constexpr double kPDriveVel = 3.09;

    constexpr units::length::meter_t  kStartPos = 4.9_m;
    //constexpr units::deg_t kStartAngularOffset = 15_deg;
    constexpr units::length::meter_t  kCenterline = 0.94_m;

}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}


