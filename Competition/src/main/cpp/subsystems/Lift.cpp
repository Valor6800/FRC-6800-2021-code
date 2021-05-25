#include "subsystems/Lift.h"

Lift::Lift() : ValorSubsystem(),
                        motor{LiftConstants::MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        brake_solenoid{LiftConstants::BRAKE_PCM_CAN_ID} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Lift::init() {
    initTable("Intake");
    table->PutNumber("Lift Speed Up", LiftConstants::DEFAULT_UP_SPD);
    table->PutNumber("Lift Speed Down", LiftConstants::DEFAULT_DOWN_SPD);

    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor.SetInverted(false);
    
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

void Lift::assignOutputs() {
    state.liftState == LiftState::MANUAL ? frc::SmartDashboard::PutString("State", "Manual") : frc::SmartDashboard::PutString("State", "Disabled");
    state.powerDown = liftTable->GetEntry("Lift Speed Down").GetDouble(0.0);
    state.powerUp = liftTable->GetEntry("Lift Speed Up").GetDouble(0.0);

    if (state.liftState == LiftState::DISABLED) {
        motor.Set(0);
        brake_solenoid.Set(true);
    } else {
        if (state.manual_input > 0)
            motor.Set(state.powerUp);
        else if (state.manual_input < 0)
            motor.Set(state.powerDown);
        else
            motor.Set(0);
        brake_solenoid.Set(false);
    }
}