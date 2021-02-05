#pragma once

#include "ValorSubsystem.h" 
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include <frc/livewindow/LiveWindow.h>

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

        void resetEncoder();

        enum ShooterState {
            DISABLED_SHOOTER,
            ON,
            OFF
        };

        enum PowerState {
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
            TRACK
        };

        struct x {
            ShooterState shooterState;
            PowerState powerState;
            TurretState turretState;

            double leftStickX;
            bool rightBumper;
            bool xButton;
            bool startButton;
            bool backButton;
            int dpad;

            double error;
            double manualPow;
            double flywheelOffsetPow;
            double limelightDistance;

            double turretTarget;
            double flywheelTarget;
            bool hoodTarget;
        } state;
    
    private:
        rev::CANSparkMax flywheelA;
        rev::CANSparkMax flywheelB;
        rev::CANSparkMax turret;
        rev::CANEncoder turretEncoder = turret.GetEncoder();

        // frc::Solenoid hood;

        nt::NetworkTableEntry manualPower;
        nt::NetworkTableEntry flywheelOffsetPower;

        frc::XboxController* operatorController;

        std::shared_ptr<nt::NetworkTable> shootTable;
        std::shared_ptr<nt::NetworkTable> limeTable;
};

#endif