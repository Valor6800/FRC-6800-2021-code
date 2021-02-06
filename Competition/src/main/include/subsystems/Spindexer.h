#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/SpeedControllerGroup.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <frc/livewindow/LiveWindow.h>

#ifndef SPINDEXER_H
#define SPINDEXER_H

class Spindexer : public ValorSubsystem {
    public:
        Spindexer();

        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        struct x {
            bool spinState;
            double spindexer_power;
            double throat_power;
        } state;

    private:
        rev::CANSparkMax motor_drum;
        rev::CANSparkMax motor_throat;
        rev::CANSparkMax motor_throat_follow;

        frc::XboxController* operatorController;
};

#endif