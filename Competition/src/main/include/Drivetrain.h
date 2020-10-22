/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h" 
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/units.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <adi/ADIS16448_IMU.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

class Drivetrain : public ValorSubsystem {
    public:
        Drivetrain();

        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        void setPower(double power);

        void resetEncoders();
        void resetOdometry(frc::Pose2d pose);
        void resetIMU();

        units::meter_t getLeftDistance();
        units::meter_t getRightDistance();
        double getHeading();
        double getTurnRate();
        frc::Pose2d getPose();
        frc::DifferentialDriveWheelSpeeds getWheelSpeeds();
        void tankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts);

        enum DrivetrainState {
            DISABLED,
            AUTO,
            MANUAL
        };

        struct x {
            DrivetrainState drivetrainState;

            double leftTrigger;
            double rightTrigger;
            double leftJoystickX;
            bool Ybutton;
            bool rightBumper;

            double leftJoystickY;
            double rightJoystickX;

            double directionX;
            double arcDirectionX;
            double directionY;
            double boostMultiplier;
            
            double straightTarget;
            double turnTarget;
            double currentLeftTarget;
            double currentRightTarget;
            int limelightState;
        } state;

        frc::DifferentialDriveKinematics kDriveKinematics;
        frc::SimpleMotorFeedforward<units::meters> kSimpleMotorFeedforward;
        frc::TrajectoryConfig kTrajectoryConfigF;
        frc::TrajectoryConfig kTrajectoryConfigR;
        frc::DifferentialDriveVoltageConstraint kDifferentialDriveVoltageConstraint;

    private:
        rev::CANSparkMax leftDriveLead;
        rev::CANSparkMax leftDriveFollowA;
        rev::CANSparkMax leftDriveFollowB;
        rev::CANSparkMax rightDriveLead;
        rev::CANSparkMax rightDriveFollowA;
        rev::CANSparkMax rightDriveFollowB;

        // rev::CANPIDController leftPIDController = leftDriveLead.GetPIDController();
        // rev::CANPIDController rightPIDController = rightDriveLead.GetPIDController();

        rev::CANEncoder leftCANEncoder = leftDriveLead.GetEncoder();
        rev::CANEncoder rightCANEncoder = rightDriveLead.GetEncoder();

        frc::ADIS16448_IMU imu{};

        frc::DifferentialDriveOdometry odometry;

        frc::XboxController* driverController;
};

#endif