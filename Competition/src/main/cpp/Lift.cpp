#include "Lift.h"

Lift::Lift() : liftMtrLeft{LiftConstants::VICTOR_ID_LEFT_Lift}, 
             liftMtrRight{LiftConstants::VICTOR_ID_RIGHT_Lift},
             liftServoLeft{LiftConstants::SERVO_ID_LEFT_Lift},
             liftServoRight{LiftConstants::SERVO_ID_RIGHT_Lift},
             operatorController(NULL) {
    initLift();
}

void Lift::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Lift::initLift() {
    
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

        state.leftServoCurrent = liftServoLeft.GetAngle();
        state.rightServoCurrent = liftServoRight.GetAngle();
    } 
    else if (operatorController->GetY(frc::GenericHID::kRightHand) < -0.05) {
        state.liftState = LiftState::RETRACT;
        state.rightJoystickY = operatorController->GetY(frc::GenericHID::kRightHand);

        state.leftServoCurrent = liftServoLeft.GetAngle();
        state.rightServoCurrent = liftServoRight.GetAngle();
    }
    else {
        state.liftState = LiftState::DISABLED;
    }
}

void Lift::assignOutputs() {
    
   if (state.liftState == LiftState::EXTEND) {
       liftServoLeft.SetAngle(LiftConstants::SERVO_UNLOCKED_ANGLE_LEFT_Lift);
       liftServoRight.SetAngle(LiftConstants::SERVO_UNLOCKED_ANGLE_RIGHT_Lift);

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
        liftServoLeft.SetAngle(LiftConstants::SERVO_LOCKED_ANGLE_LEFT_Lift);
        liftServoRight.SetAngle(LiftConstants::SERVO_LOCKED_ANGLE_RIGHT_Lift);
    }
    liftMtrLeft.Set(state.currentPower);
    liftMtrRight.Set(state.currentPower);
}