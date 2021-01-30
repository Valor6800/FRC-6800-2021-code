#pragma once

#include "ValorSubsystem.h" 
#include "Constants.h"
#include <frc/XboxController.h>

#include <rev/CANSparkMax.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>

#ifndef SHOOTER_H
#define SHOOTER_H

class Shooter : public ValorSubsystem {
    public:
        Shooter();
        
        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        enum ShooterState {
            DISABLED_SHOOTER,
            ON,
            OFF
        };

        enum PowerState {
            DISABLED_POWER,
            MANUAL_POWER,
            FENDER,
            INITIATION,
            TRENCH,
            DYNAMIC
        };

        enum TurretState {
            DISABLED_TURRET,
            MANUAL_TURRET,
            HOME,
            TRACK,
            HOLD
        };

        struct x {
            ShooterState shooterState;
            PowerState powerState;
            TurretState turretState;

            double leftStickX;
            bool rightBumper;
            bool xButton;
            bool aButton;
            bool startButton;
            bool stopButton;

            double turretTarget;
            double flywheelTarget;
        } state;
    
    private:
        rev::CANSparkMax flywheelA;
        rev::CANSparkMax flywheelB;
        rev::CANSparkMax turret;

        frc::XboxController* operatorController;

};

#endif