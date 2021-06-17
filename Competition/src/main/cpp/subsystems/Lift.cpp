#include "subsystems/Lift.h"

Lift::Lift() : ValorSubsystem(),
                        leadMotor{LiftConstants::MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        followMotor{LiftConstants::MOTOR_FOLLOW_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        brake_solenoid{LiftConstants::BRAKE_PCM_CAN_ID} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Lift::init() {
    initTable("Lift");
    table->PutNumber("Lift Speed Up", LiftConstants::DEFAULT_UP_SPD);
    table->PutNumber("Lift Speed Down", LiftConstants::DEFAULT_DOWN_SPD);

    table->PutBoolean("Safe Lift", false);

    leadMotor.RestoreFactoryDefaults();
    followMotor.RestoreFactoryDefaults();

    leadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    followMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    leadMotor.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    followMotor.Follow(leadMotor);

    leadMotor.SetInverted(false);
    
}

void Lift::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Lift::setDefaultState() {
    state.liftState = LiftState::DISABLED;
    resetState();
}

void Lift::resetState() {

}

void Lift::assessInputs() {
    if (!operatorController) {
        return;
    }

    state.liftState = operatorController->GetBumper(frc::GenericHID::kLeftHand) ? LiftState::MANUAL : LiftState::DISABLED;
    state.manual_input = std::abs(operatorController->GetY(frc::GenericHID::kRightHand)) < 0.1 ? 0 : -operatorController->GetY(frc::GenericHID::kRightHand);
}

void Lift::analyzeDashboard() {
    table->PutNumber("State", state.liftState);
    state.powerDown = table->GetNumber("Lift Speed Down", LiftConstants::DEFAULT_DOWN_SPD);
    state.powerUp = table->GetNumber("Lift Speed Up", LiftConstants::DEFAULT_UP_SPD);
    if (table->GetBoolean("Safe Speed", false)) {
        state.liftState = LiftState::SAFE;
    }
}

void Lift::assignOutputs() {
    if (state.liftState == LiftState::DISABLED) {
        leadMotor.Set(0);
        brake_solenoid.Set(false);
    } else {
        if (state.manual_input > 0) {
            if (state.liftState == LiftState::SAFE)
                leadMotor.Set(LiftConstants::SAFE_SPEED);
            else
                leadMotor.Set(state.powerUp);
         } else if (state.manual_input < 0) {
            if (state.liftState == LiftState::SAFE)
                leadMotor.Set(-LiftConstants::SAFE_SPEED);
            else
                leadMotor.Set(state.powerDown);
         }else
            leadMotor.Set(0);
        brake_solenoid.Set(true);
    }
}