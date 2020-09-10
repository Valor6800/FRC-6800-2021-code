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

        void init();

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

            double currentPower;

            double rightJoystickY;

            double leftServoCurrent;
            double rightServoCurrent;
        } state;

    private:
        frc::PWMVictorSPX liftMtrLeft;
        frc::PWMVictorSPX liftMtrRight;

        frc::Servo liftServoLeft;
        frc::Servo liftServoRight;

        frc::XboxController* operatorController;
};

#endif