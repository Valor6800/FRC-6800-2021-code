#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/SpeedControllerGroup.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <frc/livewindow/LiveWindow.h>

#ifndef INTAKE_H
#define INTAKE_H

class Intake : public ValorSubsystem {
    public:
        Intake();

        void init();
        void setControllers(frc::XboxController* controllerO, frc::XboxController* controllerD);

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        enum IntakeState {
            DISABLED,
            DEPLOY,
            RETRACT
        };

        struct x {
            IntakeState intakeState;
            double power;
            bool altState;
        } state;
    private:
        rev::CANSparkMax motor;

        frc::Compressor compressor;
        frc::DoubleSolenoid leftSolenoid;
        frc::DoubleSolenoid rightSolenoid;

        frc::XboxController* operatorController;
        frc::XboxController* driverController;

        std::shared_ptr<nt::NetworkTable> intakeTable;
};

#endif