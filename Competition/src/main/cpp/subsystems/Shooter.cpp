#include "subsystems/Shooter.h"

Shooter::Shooter() : ValorSubsystem(),
                     flywheelA{ShooterConstants::CAN_ID_FLYWHEEL_A, rev::CANSparkMax::MotorType::kBrushless},
                     flywheelB{ShooterConstants::CAN_ID_FLYWHEEL_B, rev::CANSparkMax::MotorType::kBrushless},
                     turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                     operatorController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void Shooter::init() {
    flywheelA.RestoreFactoryDefaults();
    flywheelB.RestoreFactoryDefaults();
    turret.RestoreFactoryDefaults();

    flywheelA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flywheelB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // turrent on brake?
    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // test
    flywheelA.SetInverted(false);
    flywheelB.SetInverted(true);
    turret.SetInverted(false);
}

void Shooter::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Shooter::setDefaultState() {
    state.shooterState = ShooterState::DISABLED_SHOOTER;
    state.powerState = PowerState::DISABLED_POWER;
    state.turretState = TurretState::DISABLED_TURRET;
}

void Shooter::assessInputs() {
    if (!operatorController) {
        return;
    }
}

void Shooter::assignOutputs() {

}

void Shooter::resetState() {

}