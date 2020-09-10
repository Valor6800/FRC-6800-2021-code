#include "Lift.h"

Lift::Lift() :  ValorSubsystem(),
                liftMtrLeft{LiftConstants::VICTOR_ID_LEFT_Lift}, 
                liftMtrRight{LiftConstants::VICTOR_ID_RIGHT_Lift},
                liftServoLeft{LiftConstants::SERVO_ID_LEFT_Lift},
                liftServoRight{LiftConstants::SERVO_ID_RIGHT_Lift},
                operatorController(NULL) {
}

void Lift::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Lift::init() {
    
    liftMtrLeft.SetInverted(false);
    liftMtrRight.SetInverted(false);
}

void Lift::setDefaultState() {
    state.liftState = LiftState::DISABLED;

    resetState();
}

void Lift::assessInputs() {
    // Prevent controller segfault
    if (!operatorController) {
        return;
    }

    if (operatorController->GetY(frc::GenericHID::kRightHand) > 0.05) {
        state.liftState = LiftState::EXTEND;
        state.rightJoystickY = operatorController->GetY(frc::GenericHID::kRightHand);

        state.leftServoCurrent = liftServoLeft.Get();
        state.rightServoCurrent = liftServoRight.Get();
    } 
    else if (operatorController->GetY(frc::GenericHID::kRightHand) < -0.05) {
        state.liftState = LiftState::RETRACT;
        state.rightJoystickY = operatorController->GetY(frc::GenericHID::kRightHand);

        state.leftServoCurrent = liftServoLeft.Get();
        state.rightServoCurrent = liftServoRight.Get();
    }
    else {
        state.liftState = LiftState::DISABLED;
        state.rightJoystickY = 0;
    }
}

void Lift::assignOutputs() {
    
   if (state.liftState == LiftState::EXTEND) {
       liftServoLeft.Set(LiftConstants::SERVO_UNLOCKED_ANGLE_LEFT_Lift);
       liftServoRight.Set(LiftConstants::SERVO_UNLOCKED_ANGLE_RIGHT_Lift);

        if (state.leftServoCurrent == LiftConstants::SERVO_UNLOCKED_ANGLE_LEFT_Lift && state.rightServoCurrent == LiftConstants::SERVO_UNLOCKED_ANGLE_RIGHT_Lift)
        {
            state.currentPower = state.rightJoystickY;
        }
        else {
            state.currentPower = 0;
        } 
    }
    else if (state.liftState == LiftState::RETRACT) {

        if (state.leftServoCurrent == LiftConstants::SERVO_LOCKED_ANGLE_LEFT_Lift && state.rightServoCurrent == LiftConstants::SERVO_LOCKED_ANGLE_RIGHT_Lift)
        {
            state.currentPower = state.rightJoystickY;
        }
        else {
            state.currentPower = 0;
        } 
    }
    else {
        liftServoLeft.Set(LiftConstants::SERVO_LOCKED_ANGLE_LEFT_Lift);
        liftServoRight.Set(LiftConstants::SERVO_LOCKED_ANGLE_RIGHT_Lift);
    }
    liftMtrLeft.Set(state.currentPower);
    liftMtrRight.Set(state.currentPower);
}