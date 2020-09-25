#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Timer.h>

#ifndef INTAKE_H
#define INTAKE_H

class Intake : public ValorSubsystem {
    public:
        Intake();
        void setController(frc::XboxController*, frc::XboxController*);
        
        void init();
        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        enum IntakeState {
            DISABLED, 
            IN, 
            OUT
        };

        struct x {
            IntakeState intakeState;

            //frc::Timer timer;
            
            double currentPower;
            //double currentTime;
            
        } state;

    private:
        frc::PWMVictorSPX intakeMtr;

        frc::XboxController* operatorController;
        frc::XboxController* driverController;
};

#endif