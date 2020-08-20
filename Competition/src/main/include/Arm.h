#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <ctre/Phoenix.h>

#ifndef ARM_H
#define ARM_H

class Arm : public ValorSubsystem {
    public:
        Arm();
        void setController(frc::XboxController*);

        void initArm();

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void setDisengage(bool disengage);
        void resetState();

        enum ArmState {
            DISABLED, 
            DISENGAGE, 
            MANUAL
        };

        struct x {
            ArmState armState;

            frc::Timer timer;
            bool disengage;
            double currentPower;
            double currentTime;
            double step1_startTime;
            double step2_startTime;

            double leftJoystickY;
        } state;

    private:
        TalonSRX armMtrLeft;
        TalonSRX armMtrRight;

        frc::XboxController* operatorController;
};

#endif