/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() : ValorSubsystem(),
                           leftA{DriveConstants::CAN_ID_LEFT_A},
                           leftB{DriveConstants::CAN_ID_LEFT_B},
                           rightA{DriveConstants::VICTOR_ID_RIGHT_A},
                           rightB{DriveConstants::VICTOR_ID_RIGHT_B},
                           rightDrive{rightA, rightB},
                           driverController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void Drivetrain::init() {
    leftA.SetInverted(false);
    leftB.SetInverted(false);
    rightA.SetInverted(true);
    rightB.SetInverted(true);
}

void Drivetrain::setController(frc::XboxController* controller) {
    driverController = controller;
}

void Drivetrain::setDefaultState() {
    state.drivetrainState = DrivetrainState::DISABLED;

    resetState();
}

void Drivetrain::assessInputs() {
    if (!driverController) {
        return;
    }

    if (std::abs(driverController->GetY(frc::GenericHID::kLeftHand)) > DriveConstants::kDeadbandY ||
        std::abs(driverController->GetX(frc::GenericHID::kRightHand)) > DriveConstants::kDeadbandX) {
            state.drivetrainState = DrivetrainState::MANUAL;

            state.leftStickY = -driverController->GetY(frc::GenericHID::kLeftHand);
            state.rightStickX = driverController->GetX(frc::GenericHID::kRightHand);

            frc::SmartDashboard::PutNumber("Left Stick Y", state.leftStickY);
            frc::SmartDashboard::PutNumber("Right Stick X", state.rightStickX);

            state.directionX = state.rightStickX / std::abs(state.rightStickX);

            frc::SmartDashboard::PutNumber("Direction X", state.directionX);

            state.straightTarget = state.leftStickY;
            state.turnTarget = std::pow(state.rightStickX, 2) * state.directionX * DriveConstants::kArcTurnMultipler;

            frc::SmartDashboard::PutNumber("Straight target", state.straightTarget);
            frc::SmartDashboard::PutNumber("Turn target", state.turnTarget);

            if (std::abs(state.leftStickY) < DriveConstants::kDeadbandY) {
                state.straightTarget = 0;
            }

            if (std::abs(state.rightStickX) < DriveConstants::kDeadbandX) {
                state.turnTarget = 0;
            }

            state.currentLeftTarget = state.straightTarget + state.turnTarget;
            state.currentRightTarget = state.straightTarget - state.turnTarget;

            frc::SmartDashboard::PutNumber("Left target", state.currentLeftTarget);
            frc::SmartDashboard::PutNumber("Right target", -state.currentRightTarget);
    }
    else {
        state.drivetrainState = DrivetrainState::DISABLED;
    }
}

void Drivetrain::assignOutputs() {
    state.drivetrainState == DrivetrainState::MANUAL ? frc::SmartDashboard::PutString("State", "Manual") : frc::SmartDashboard::PutString("State", "Disabled");

    if (state.drivetrainState == DrivetrainState::MANUAL) {
        leftA.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, state.currentLeftTarget);
        leftB.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, state.currentLeftTarget);
        rightDrive.Set(-state.currentRightTarget);
    }
    else {
        setPower(0);
    }
}

void Drivetrain::resetState() {

}

void Drivetrain::setPower(double power) {
    leftA.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
    leftB.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
    rightDrive.Set(-power);
}