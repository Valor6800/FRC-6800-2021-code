/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

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

    constexpr static double limeLightKP = 0.005;
}

namespace ShooterConstants {
    constexpr static int CAN_ID_FLYWHEEL_FOLLOW = 10;
    constexpr static int CAN_ID_FLYWHEEL_LEAD = 8;
    constexpr static int CAN_ID_TURRET = 9;
    constexpr static int SOLENOID_ID_SHOOTER = 1;

    // Deadband for the left joystick x axis
    constexpr static double kDeadband = 0.05;

    constexpr static double TURRET_SPEED_MULTIPLIER = 0.5;

    constexpr static int dpadUp = 0;
    constexpr static int dpadRight = 90;
    constexpr static int dpadDown = 180;
    constexpr static int dpadLeft = 270;

    // Turret input P control value
    constexpr static double turretKP = 1/16.0;
    constexpr static double turretKQ = 8;
    // Turret power deadband. Minimum power required to move the turret
    constexpr static double pDeadband = 0.01;
    constexpr static double pSoftDeadband = 0.06;
    // Turret limelight P control value
    constexpr static double limelightTurnKp = 1/24.0;
    constexpr static double limelightTurnKq = 4;
    constexpr static double limelightDistKp = 0.2;


    // Fender shot successful: throat 50%, shooter 75%
    // initiation: shooter 65%
    // trench: 70%
    constexpr static int fenderPower = 4000; // 3200-3300
    constexpr static int initiationPower = 3800;//3900; // 2800-2900
    constexpr static int trenchPower = 3900; // 3200- 3300
    constexpr static int defaultManualPower = 3500; // 3200

    constexpr static double shooterKP = 0.001;
    constexpr static double shooterKI = 0.00001;
    constexpr static double shooterKD = 0;
    constexpr static double shooterKIZ = 0;
    constexpr static double shooterKFF = 0.00023;
    constexpr static double shooterMax = 1;
    constexpr static double MaxRPM = 5700;

    //turret encoder
    //encoder is on outputshaft of neo
    //5 rotations of neo ~= 20 degree of rotation
    //facing front of robot, counter clockwise is positive, clockwise is negative

    constexpr static double homePosition = 0;
    constexpr static double homeFrontPosition = 153;

    // Encoder ticks off of center
    // 192 (gear ration) * angle ratio (ex. 1/2 for 180 deg)
    constexpr static double limitLeft = homePosition + 200; // 20;
    constexpr static double limitRight = homePosition - 15; // -15;
}

namespace SpindexerConstants {
    constexpr static int CAN_ID = 13;
    constexpr static int CAN_ID_THROAT = 11;
    constexpr static int CAN_ID_THROAT_FOLLOW = 12;

    constexpr static int CACHE_SIZE = 20;
    constexpr static double JAM_CURRENT = 9;
    constexpr static double DRUM_GEAR_RATIO = 67.5;
    constexpr static double UMJAM_ROTATIONS = DRUM_GEAR_RATIO / 4.5;

    constexpr static double left_trigger_deadband = 0.05;
    constexpr static double default_throat_spd = 0.8;
    constexpr static double default_drum_spd = 0.25;

    constexpr static double high_spd_drum = 0.7;
    constexpr static double high_spd_drum_fender = 0.7;
    constexpr static double high_spd_drum_initiation = 0.7;
    constexpr static double high_spd_drum_trench = 0.7;
}

namespace LiftConstants {
    constexpr static int MOTOR_CAN_ID = 5;
    constexpr static int MOTOR_FOLLOW_CAN_ID = 6;
    constexpr static int BRAKE_PCM_CAN_ID = 2;

    constexpr static double DEFAULT_UP_SPD = -0.33;
    constexpr static double DEFAULT_DOWN_SPD = 1;

    constexpr static double SAFE_SPEED = -0.1;
}

namespace IntakeConstants {
    constexpr static double DEFAULT_ROLLER_SPD = 1;

    constexpr static int MOTOR_CAN_ID = 14;

    constexpr static int SOLENOID_FORWARD_PCM_CAN_ID = 0;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;

    constexpr static double TrACKING_OFFSET = 2; 
}

namespace RamseteConstants {

    constexpr auto kMaxForwardSpeed = 2.12_mps;
    constexpr auto kMaxReverseSpeed = 4.12_mps;
    constexpr auto kMaxAcceleration = 4_mps_sq;

    constexpr double kRamseteB = 2;
    constexpr double kRamseteZeta = 0.7;

    // Convert rotations to meters using gear ratio and wheel diameter (converted to meters)
    constexpr double kPositionConversionFactor = DriveConstants::kWheelDiameterInches * 0.0254 / DriveConstants::kGearRatio * M_PI;
    // Convert rotations per minute to meters per second 
    constexpr double kVelocityConversionFactor = kPositionConversionFactor / 60;

    constexpr bool kGyroReversed = false;

    constexpr auto kS = 0.21_V;
    constexpr auto kV = 1.84 * 1_V * 1_s / 1_m;
    constexpr auto kA = 0.322 * 1_V * 1_s * 1_s / 1_m;

    constexpr double kPDriveVel = 2.71;

    constexpr units::length::meter_t  kStartPos = 4.9_m;
    constexpr units::length::meter_t  kCenterline = 0.94_m;
    //constexpr units::deg_t kStartAngularOffset = 15_deg;
}

#endif
