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

#include "subsystems/Shooter.h"

#ifndef SPINDEXER_H
#define SPINDEXER_H

class Spindexer : public ValorSubsystem {
    public:
        Spindexer();

        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();

        void resetState();

        enum DrumState {
            STOPPED,
            LOW,
            UNJAM,
            HIGH
        };

        struct x {
            double spindexer_power;
            double throat_lead_power;
            double throat_follow_power;

            DrumState drumState;

            std::vector<double> current_cache;
            int current_cache_index;
            double initial_jam_position;

            double instCurrent;
            
            bool flywheelState;
            Shooter::PowerState powerState;
        } state;

    private:
        rev::CANSparkMax motor_drum;
        rev::CANSparkMax motor_throat;
        rev::CANSparkMax motor_throat_follow;

        rev::CANEncoder drum_encoder = motor_drum.GetEncoder();

        frc::XboxController* operatorController;

        std::shared_ptr<nt::NetworkTable> intakeTable;
        std::shared_ptr<nt::NetworkTable> shooterTable;

        void calcCurrent();
};

#endif