#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

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
        void analyzeDashboard();
        void assignOutputs();

        void resetState();

        enum DeployState {
            RETRACT,
            DEPLOY
        };

        struct x {
            DeployState deployState;
            bool intakeState;
            double power;
        } state;
    private:
        rev::CANSparkMax motor;

        frc::Solenoid solenoid;

        frc::XboxController* operatorController;
        frc::XboxController* driverController;
};

#endif