#include "subsystems/Spindexer.h"

Spindexer::Spindexer() : ValorSubsystem(),
                        motor{SpindexerConstants::CAN_ID} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    spinTable = nt::NetworkTableInstance::GetDefault().GetTable("spindexer");
    spinTable->GetEntry("Speed?").SetDouble(0.0);
}

void Spindexer::init() {
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
        double power2 = spinTable->GetEntry("Speed?").GetDouble(0.0);
        frc::SmartDashboard::PutNumber("Spindexer Speed", power2);
        state.powah = 1.0;
    } else {
        state.spinState = SpindexerState::DISABLED;
    }
}

void Spindexer::assignOutputs() {
    state.spinState == SpindexerState::ENABLED ? frc::SmartDashboard::PutString("State", "Enabled") : frc::SmartDashboard::PutString("State", "Disabled");

    if (state.spinState == SpindexerState::ENABLED) {
        motor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, state.powah);
    } else {
        motor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, 0);
    }
}