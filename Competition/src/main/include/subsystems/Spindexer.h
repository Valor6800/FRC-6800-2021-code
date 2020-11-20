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

        enum SpindexerState {
            DISABLED,
            ENABLED
        };

        struct x {
            SpindexerState spinState;
            double power;
        } state;
    private:
        rev::CANSparkMax motor;

        frc::XboxController* driverController;

        std::shared_ptr<nt::NetworkTable> spinTable;
};

#endif