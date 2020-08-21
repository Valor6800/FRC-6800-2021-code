#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Servo.h>

#ifndef LIFT_H
#define LIFT_H

class Lift : public ValorSubsystem {
    public:
        Lift();
        void setController(frc::XboxController*);

        void initLift();

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void setDisengage(bool disengage);
        void resetState();

        enum LiftState {
            DISABLED,
            RETRACT,
            EXTEND
        };

        struct x {
            LiftState liftState;

            frc::Timer timer;
            double currentPower;

            double rightJoystickY;

            double leftServoCurrent;
            double rightServoCurrent;

            double leftServoTarget;
            double rightServoTarget;  
        } state;

    private:
        frc::PWMVictorSPX liftMtrLeft;
        frc::PWMVictorSPX liftMtrRight;

        frc::Servo liftServoLeft;
        frc::Servo liftServoRight;

        frc::XboxController* operatorController;
};

#endif