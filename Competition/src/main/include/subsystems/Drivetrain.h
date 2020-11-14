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
#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
            MANUAL
        };

        struct x {
            DrivetrainState drivetrainState;

            double leftStickY;
            double rightStickX;

            double directionX;

            double straightTarget;
            double turnTarget;
            double currentLeftTarget;
            double currentRightTarget;
        } state;
    
    private:
        VictorSPX leftA;
        VictorSPX leftB;
        frc::PWMVictorSPX rightA;
        frc::PWMVictorSPX rightB;

        frc::SpeedControllerGroup rightDrive;

        frc::XboxController* driverController;
};

#endif