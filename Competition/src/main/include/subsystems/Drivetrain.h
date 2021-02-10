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
#include <frc/PWMVictorSPX.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <ctre/Phoenix.h>
#include <adi/ADIS16448_IMU.h>

#include <frc/SpeedControllerGroup.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <units/units.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

class Drivetrain : public ValorSubsystem {
    public:
        Drivetrain();

        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        
        double GetEncAvgDistance();
        double GetHeading();
        double GetTurnRate();

        units::meter_t GetLeftDistance();
        units::meter_t GetRightDistance();
        frc::Pose2d GetPose();
        frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
        
        void ResetEncoders();
        void ResetOdometry(frc::Pose2d pose);
        void ResetIMU();
        void resetState();

        void TankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts);
        void setPower(double power);

        enum DriveModeState {
            ARCADE,
            ROCKET_LEAGUE
        };

        struct x {
            bool tracking;
            DriveModeState driveModeState;

            bool yButton;

            double leftStickY;
            double rightStickX;

            double leftStickX;
            double rightTrigger;
            double leftTrigger;

            double directionX;
            double directionY;
            double boostMultiplier;

            double straightTarget;
            double turnTarget;
            double currentLeftTarget;
            double currentRightTarget;
        } state;

        frc::DifferentialDriveKinematics kDriveKinematics;
        frc::SimpleMotorFeedforward<units::meters> kSimpleMotorFeedforward;
        frc::TrajectoryConfig kTrajectoryConfigForward;
        frc::TrajectoryConfig kTrajectoryConfigReverse;
        frc::DifferentialDriveVoltageConstraint kDifferentialDriveVoltageConstraint;
    
    private:
        rev::CANSparkMax leftDriveLead;
        rev::CANSparkMax leftDriveFollow;
        rev::CANSparkMax rightDriveLead;
        rev::CANSparkMax rightDriveFollow;

        frc::XboxController* driverController;

        frc::ADIS16448_IMU imu{};

        rev::CANPIDController leftPIDController = leftDriveLead.GetPIDController();
        rev::CANPIDController rightPIDController = rightDriveLead.GetPIDController();

        rev::CANEncoder leftEncoder = leftDriveLead.GetEncoder();
        rev::CANEncoder rightEncoder = rightDriveLead.GetEncoder();

       frc::DifferentialDriveOdometry m_odometry;
};

#endif