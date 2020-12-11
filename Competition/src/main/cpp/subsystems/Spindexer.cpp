#include "subsystems/Spindexer.h"

Spindexer::Spindexer() : ValorSubsystem(),
                        motor{SpindexerConstants::CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        throat_motor(4, rev::CANSparkMax::MotorType::kBrushless) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    spinTable = nt::NetworkTableInstance::GetDefault().GetTable("spindexer");
    spinTable->GetEntry("Spindexer Speed").SetDouble(0.0);
}

void Spindexer::init() {
    throat_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor.SetInverted(true);
    throat_motor.SetInverted(true);
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

    if (driverController->GetBumper(frc::GenericHID::kRightHand)) {
        state.spinState = SpindexerState::POWERED;
        // state.power = spinTable->GetEntry("Spindexer Speed").GetDouble(0.0);
        state.power = .75;
        state.throat_power = .5;
    } else {
        state.spinState = SpindexerState::COASTING;
        state.power = .25;
        state.throat_power = 0.0;
    }
}

void Spindexer::assignOutputs() {
    if (state.spinState == SpindexerState::DISABLED) {
        frc::SmartDashboard::PutString("StateSpindexer", "DISABLED");
    } else {
        state.spinState == SpindexerState::COASTING ? frc::SmartDashboard::PutString("StateSpindexer", "COASTING") : frc::SmartDashboard::PutString("StateSpindexer", "POWERED");
    }

    if (state.spinState == SpindexerState::COASTING || state.spinState == SpindexerState::POWERED) {
        motor.Set(state.power);
        throat_motor.Set(state.throat_power);
    } else {
        motor.Set(0);
        throat_motor.Set(0);   
    }
}