#include "Arm.h"

Arm::Arm() : armMtrLeft{ArmConstants::TALON_ID_LEFT_ARM}, 
             armMtrRight{ArmConstants::TALON_ID_RIGHT_ARM},
             operatorController(NULL) {
    initArm();
}

void Arm::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Arm::initArm() {
    armMtrLeft.ConfigFactoryDefault();
    armMtrRight.ConfigFactoryDefault();

    armMtrLeft.SetNeutralMode(NeutralMode::Brake);
    armMtrRight.SetNeutralMode(NeutralMode::Brake);

    armMtrLeft.SetInverted(false);
    armMtrRight.SetInverted(false);
}

void Arm::setDefaultState() {
    state.armState = ArmState::DISABLED;

    resetState();
}

void Arm::assessInputs() {
    // Prevent controller segfault
    if (!operatorController) {
        return;
    }

    if (std::abs(operatorController->GetY(frc::GenericHID::kLeftHand)) > 0.05) {
        state.armState = ArmState::MANUAL;
        state.leftJoystickY = operatorController->GetY(frc::GenericHID::kLeftHand);
    }
}

void Arm::assignOutputs() {
    if (state.disengage) {
        state.armState = ArmState::DISENGAGE;
    }

    if (state.armState == ArmState::DISENGAGE) {
        state.currentTime = state.timer.GetFPGATimestamp();

        if (state.step1_startTime == -1) {
            state.step1_startTime = state.currentTime;
        }
        else if (state.currentTime - state.step1_startTime <= 0.2) {
            state.currentPower = 0.4;
        }
        else if (state.step2_startTime == -1) {
            state.step2_startTime = state.currentTime;
        }
        else if (state.currentTime - state.step2_startTime <= 0.3) {
            state.currentPower = 0.042;
        }
        else {
            state.currentPower = 0;
        }
    }
    else if (state.armState == ArmState::MANUAL) {
        if (std::abs(state.leftJoystickY) <= 0.05) {
            state.currentPower = 0;
        }
        else if (state.leftJoystickY < -0.05) {
            state.currentPower = -0.5;
        }
        else if (state.leftJoystickY > 0.05 && state.leftJoystickY < 0.85) {
            state.currentPower = -0.042;
        }
        else {
            state.currentPower = 0.1;
        }
    }
    else {
        state.currentPower = 0;
    }
    armMtrLeft.Set(ControlMode::PercentOutput, state.currentPower);
    armMtrRight.Set(ControlMode::PercentOutput, state.currentPower);
}

void Arm::resetState() {
    state.disengage = false;
    state.step1_startTime = -1;
    state.step2_startTime = -1;
}