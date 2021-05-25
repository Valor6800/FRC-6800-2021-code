#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogPotentiometer.h>
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

#ifndef LIFT_H
#define LIFT_H

class Lift : public ValorSubsystem {
    public:
        Lift();

        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        enum LiftState {
            DISABLED,
            MANUAL
        };

        struct x {
            LiftState liftState;
            double manual_input;
            double powerDown;
            double powerUp;
            bool locked;
        } state;
    private:
        rev::CANSparkMax motor;
        frc::Solenoid brake_solenoid;

        frc::XboxController* operatorController;

        std::shared_ptr<nt::NetworkTable> liftTable;
};

#endif