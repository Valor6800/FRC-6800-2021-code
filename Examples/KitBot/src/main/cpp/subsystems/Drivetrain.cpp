/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() : ValorSubsystem(),
                           leftA{DriveConstants::VICTOR_ID_LEFT_A},
                           leftB{DriveConstants::VICTOR_ID_LEFT_B},
                           rightA{DriveConstants::VICTOR_ID_RIGHT_A},
                           rightB{DriveConstants::VICTOR_ID_RIGHT_B},
                           leftDrive{leftA, leftB},
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

            state.leftStickY = driverController->GetY(frc::GenericHID::kLeftHand);
            state.rightStickX = driverController->GetX(frc::GenericHID::kRightHand);

            state.straightTarget = state.leftStickY;
            state.turnTarget = std::pow(state.rightStickX, 2) * DriveConstants::kArcTurnMultipler;

            if (std::abs(state.leftStickY) < DriveConstants::kDeadbandY) {
                state.straightTarget = 0;
            }

            if (std::abs(state.rightStickX) < DriveConstants::kDeadbandX) {
                state.turnTarget = 0;
            }

            state.currentLeftTarget = state.straightTarget - state.turnTarget;
            state.currentRightTarget = state.straightTarget + state.turnTarget;
    }
    else {
        state.drivetrainState = DrivetrainState::DISABLED;
    }
}

void Drivetrain::assignOutputs() {
    state.drivetrainState == DrivetrainState::MANUAL ? frc::SmartDashboard::PutString("State", "Manual") : frc::SmartDashboard::PutString("State", "Disabled");

    if (state.drivetrainState == DrivetrainState::MANUAL) {
        leftDrive.Set(state.currentLeftTarget);
        rightDrive.Set(state.currentRightTarget);
    }
    else {
        setPower(0);
    }
}

void Drivetrain::resetState() {

}

void Drivetrain::setPower(double power) {
    leftDrive.Set(power);
    rightDrive.Set(power);
}