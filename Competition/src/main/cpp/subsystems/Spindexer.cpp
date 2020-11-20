#include "subsystems/Spindexer.h"

Spindexer::Spindexer() : ValorSubsystem(),
                        motor{SpindexerConstants::CAN_ID, rev::CANSparkMax::MotorType::kBrushless} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    spinTable = nt::NetworkTableInstance::GetDefault().GetTable("spindexer");
    spinTable->GetEntry("Spindexer Speed").SetDouble(0.0);
}

void Spindexer::init() {
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor.SetInverted(false);
}

void Spindexer::setController(frc::XboxController* controller) {
    driverController = controller;
}

void Spindexer::setDefaultState() {
    state.spinState = SpindexerState::DISABLED;
    resetState();
}

void Spindexer::resetState() {

}

void Spindexer::assessInputs() {
    if (!driverController) {
        return;
    }

    if (driverController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.spinState = SpindexerState::ENABLED;
        state.power = spinTable->GetEntry("Spindexer Speed").GetDouble(0.0);
    } else {
        state.spinState = SpindexerState::DISABLED;
    }
}

void Spindexer::assignOutputs() {
    state.spinState == SpindexerState::ENABLED ? frc::SmartDashboard::PutString("State", "Enabled") : frc::SmartDashboard::PutString("State", "Disabled");

    if (state.spinState == SpindexerState::ENABLED) {
        motor.Set(state.power);
    } else {
        motor.Set(0);
    }
}