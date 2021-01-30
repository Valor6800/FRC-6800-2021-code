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
#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

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

        enum DrivetrainState {
            DISABLED,
            MANUAL,
            AUTO
        };

        void setState(DrivetrainState _state);

        enum DriveModeState {
            ARCADE,
            ROCKET_LEAGUE
        };

        struct x {
            DrivetrainState drivetrainState;
            DriveModeState driveModeState;

            bool yButton;

            double leftStickY;
            double rightStickX;

            double leftStickX;
            double rightTrigger;
            double leftTrigger;
            bool rightBumper;

            double directionX;
            double directionY;
            double boostMultiplier;

            double straightTarget;
            double turnTarget;
            double currentLeftTarget;
            double currentRightTarget;
        } state;
    
    private:
        rev::CANSparkMax leftDriveLead;
        rev::CANSparkMax leftDriveFollow;
        rev::CANSparkMax rightDriveLead;
        rev::CANSparkMax rightDriveFollow;

        frc::XboxController* driverController;
};

#endif