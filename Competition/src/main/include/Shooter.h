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
#include <rev/CANSparkMax.h>
#include <frc/Servo.h>
#include <frc/AnalogPotentiometer.h>

#ifndef SHOOTER_H
#define SHOOTER_H

class Shooter : public ValorSubsystem {
    public:
        Shooter();
        void setController(frc::XboxController* controller);
        
        void init();

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();
        
        enum ShooterState {
            DISABLED,
            SHOOTING
        };
        
        enum HoodState {
            HOOD_UP,
            HOOD_DOWN
        };
        
        struct x {
            double currentTarget;
            double previousTarget;

            ShooterState shooterState;
            HoodState hoodState;
        } state;

    private:
        rev::CANSparkMax shooterMtr;

        rev::CANEncoder shooterEncoder = shooterMtr.GetEncoder();

        rev::CANPIDController shooterPID = shooterMtr.GetPIDController();

        // frc::Servo hoodServoLeft;
        // frc::Servo hoodServoRight;

        // frc::AnalogPotentiometer hoodPotentiometer;
        
        frc::XboxController* operatorController;
};

#endif

