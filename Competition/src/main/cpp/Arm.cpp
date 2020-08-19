#include "Arm.h"

Arm::Arm() : operator_controller(NULL),
             armMtrLeft{ArmConstants::TALON_ID_LEFT_ARM}, 
             armMtrRight{ArmConstants::TALON_ID_RIGHT_ARM} {
    InitArm();
}

void Arm::setController(frc::XboxController* controller)
{
    operator_controller = controller;
}

Arm& Arm::GetInstance() {
    static Arm instance;
    return instance;
}

void Arm::Periodic() {
    assessInputs();
    assignOutputs();
}

void Arm::InitArm() {
    armMtrLeft.ConfigFactoryDefault();
    armMtrRight.ConfigFactoryDefault();

    armMtrLeft.SetNeutralMode(NeutralMode::Brake);
    armMtrRight.SetNeutralMode(NeutralMode::Brake);

    armMtrLeft.SetInverted(false);
    armMtrRight.SetInverted(false);

    state.disengage = true;
    state.step1_start_time = -1;
    state.step2_start_time = -1;
}

void Arm::setDefaultState() {
    state.arm_state = ArmState::DISABLED;
}

void Arm::assessInputs() {
    // Prevent controller segfault
    if (!operator_controller)
        return;

    if (state.disengage) {
        state.arm_state = ArmState::DISENGAGE;
    }
    else if (std::abs(operator_controller->GetY(frc::GenericHID::kLeftHand)) > 0.05) {
        state.arm_state = ArmState::MANUAL;
    }
}

void Arm::assignOutputs() {
    if (state.arm_state == ArmState::DISENGAGE) {
        state.curr_time = state.timer.GetFPGATimestamp();

        if (state.step1_start_time == -1) {
            state.step1_start_time = state.curr_time;
        }
        else if (state.curr_time - state.step1_start_time <= 0.2) {
            state.current_power = 0.4;
        }
        else if (state.step2_start_time == -1) {
            state.step2_start_time = state.curr_time;
        }
        else if (state.curr_time - state.step2_start_time <= 0.3) {
            state.current_power = 0.042;
        }
        else {
            state.current_power = 0;

            // reset code
            // should reset code be in setDefaultState()?
            state.disengage = false;
            state.step1_start_time = -1;
            state.step2_start_time = -1;
        }
    }
    else if (state.arm_state == ArmState::MANUAL) {
        if (std::abs(operator_controller->GetY(frc::GenericHID::kLeftHand)) <= 0.05) {
            state.current_power = 0;
        }
        else if (operator_controller->GetY(frc::GenericHID::kLeftHand) < -0.05) {
            state.current_power = -0.5;
        }
        else if (operator_controller->GetY(frc::GenericHID::kLeftHand) > 0.05 && operator_controller->GetY(frc::GenericHID::kLeftHand) < 0.85) {
            state.current_power = -0.042;
        }
        else {
            state.current_power = 0.1;
        }
    }
    else {
        state.current_power = 0;
    }
    armMtrLeft.Set(ControlMode::PercentOutput, state.current_power);
    armMtrRight.Set(ControlMode::PercentOutput, state.current_power);

}