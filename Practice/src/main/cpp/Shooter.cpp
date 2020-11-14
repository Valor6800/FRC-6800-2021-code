/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Shooter.h"

Shooter::Shooter() : ValorSubsystem(),
                     shooterMtr{ShooterConstants::CAN_ID_SHOOTER, rev::CANSparkMax::MotorType::kBrushless},
                     operatorController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void Shooter::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Shooter::init() {
    shooterMtr.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    shooterPID.SetP(ShooterConstants::kP);
    shooterPID.SetI(ShooterConstants::kI);
    shooterPID.SetD(ShooterConstants::kD);
    shooterPID.SetIZone(ShooterConstants::kIz);
    shooterPID.SetFF(ShooterConstants::kFF);
    shooterPID.SetOutputRange(ShooterConstants::MIN_OUTPUT, ShooterConstants::MAX_OUTPUT);

    shooterMtr.SetInverted(true);
}

void Shooter::setDefaultState() {
    state.shooterState = ShooterState::DISABLED;
    state.hoodState = HoodState::HOOD_DOWN;

    resetState();
}

void Shooter::assessInputs() {
    if (!operatorController) {
        return;
    }

    if (operatorController->GetStartButton()) {
        state.shooterState = ShooterState::SHOOTING;
        state.hoodState = HoodState::HOOD_UP;
    }

    // overrides shoot
    if (operatorController->GetBackButton()) {
        state.shooterState = ShooterState::DISABLED;
        state.hoodState = HoodState::HOOD_DOWN;
    }
}

void Shooter::assignOutputs() {
    if (state.shooterState == ShooterState::SHOOTING) {
        state.currentTarget = ShooterConstants::SHOOT_POWER * ShooterConstants::MAX_RPM;
    } 
    else {
        state.currentTarget = 0;
    }
    
    shooterPID.SetReference(state.currentTarget, rev::ControlType::kVelocity);
}

void Shooter::resetState() {
    state.currentTarget = 0;
}
