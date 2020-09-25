#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Timer.h>

#ifndef HOPPER_H
#define HOPPER_H

class Hopper : public ValorSubsystem {
    public:
        Hopper();
        void setController(frc::XboxController*, frc::XboxController*);
        
        void init();
        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        enum HopperState {
            DISABLED, 
            FORWARD
        };

        struct x {
            HopperState hopperState;
            
            double currentHopperPower;
            double currentThroatPower;
        } state;

    private:
        frc::PWMVictorSPX hopperMtr;
        frc::PWMVictorSPX throatMtr;

        frc::XboxController* operatorController;
        frc::XboxController* driverController;
};

#endif
