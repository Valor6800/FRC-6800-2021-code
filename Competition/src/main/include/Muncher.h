#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>

#ifndef MUNCHER_H
#define MUNCHER_H

class Muncher : public ValorSubsystem {
    public:
        Muncher();
        void setController(frc::XboxController*);

        void initMuncher();

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        enum MuncherState {
            DISABLED,
            MUNCH
        };

        struct x {
            MuncherState m_state;

            double target;

            double buttonY;
        } state;

    private:
        frc::XboxController* operatorController;

        frc::PWMVictorSPX munchMtr;
};

#endif