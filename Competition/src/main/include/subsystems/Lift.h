#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>

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
            RETRACT,
            EXTEND
        };

        struct x {
            LiftState liftState;
            double power;
            double target; //This is the motion profiling variable
            bool limit;
        } state;
    private:
        rev::CANSparkMax leftMotor;
        rev::CANSparkMax rightMotor;

        frc::XboxController* operatorController;

        frc::DigitalInput limitSwitch;

        std::shared_ptr<nt::NetworkTable> liftTable;
};

#endif