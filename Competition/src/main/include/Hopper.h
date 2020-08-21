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
        void setController(frc::XboxController*);

        void initHopper();

        void setDefaultState();
        void assessInputs();
        void assignOutputs();


        enum HopperState {
            DISABLED, 
            FORWARD, 
            REVERSE
        };

        struct x {
            HopperState hopperState;

            frc::Timer timer;
            
            double currentPower;
            double currentTime;
            

        } state;

    private:
        frc::PWMVictorSPX hopperMtr;
        frc::PWMVictorSPX throatMtr;

        frc::XboxController* operatorController;
        frc::XboxController* driverController;
};

#endif
