#include "subsystems/Spindexer.h"

Spindexer::Spindexer() : ValorSubsystem(),
                        motor_drum{SpindexerConstants::CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        motor_throat{SpindexerConstants::CAN_ID_THROAT, rev::CANSparkMax::MotorType::kBrushless},
                        motor_throat_follow{SpindexerConstants::CAN_ID_THROAT_FOLLOW, rev::CANSparkMax::MotorType::kBrushless} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    spinTable = nt::NetworkTableInstance::GetDefault().GetTable("spindexer");
    spinTable->GetEntry("Spindexer Speed").SetDouble(0.0);
    spinTable->GetEntry("Throat Speed").SetDouble(0.0);
}

void Spindexer::init() {
    motor_drum.RestoreFactoryDefaults();
    motor_throat.RestoreFactoryDefaults();
    motor_throat_follow.RestoreFactoryDefaults();
    
    motor_drum.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_drum.SetInverted(false);
    
    motor_throat.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    //motor_throat.SetInverted(false);

    motor_throat.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    motor_throat_follow.Follow(rev::CANSparkMax::kFollowerDisabled, false);

    motor_throat_follow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_throat_follow.SetInverted(true);
    //motor_throat_follow.Follow(motor_throat);
    

}

void Spindexer::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Spindexer::setDefaultState() {
    state.spinState = SpindexerState::IDLE;
    resetState();
}

void Spindexer::resetState() {

}

void Spindexer::assessInputs() {
    if (!operatorController) {
        return;
    }
//(std::abs(state.rightStickX) < DriveConstants::kDeadbandX)
    if (std::abs(operatorController->GetTriggerAxis(frc::GenericHID::kLeftHand)) > SpindexerConstants::left_trigger_deadband) {
        state.spinState = SpindexerState::SHOOTING;
    } else {
        state.spinState = SpindexerState::IDLE;
    }
}

void Spindexer::assignOutputs() {
    state.spinState == SpindexerState::SHOOTING ? frc::SmartDashboard::PutString("State", "SHOOTING") : frc::SmartDashboard::PutString("State", "IDLE");
    state.spindexer_power = spinTable->GetEntry("Spindexer Speed").GetDouble(0.0);
    state.throat_power = spinTable->GetEntry("Throat Speed").GetDouble(0.0);

    if (state.spinState == SpindexerState::SHOOTING) {        
        motor_drum.Set(state.spindexer_power);       
        motor_throat.Set(state.throat_power * -1); //reverse direction
        motor_throat_follow.Set(state.throat_power);
    } else {
        motor_drum.Set(state.spindexer_power*0.4);
        motor_throat.Set(0);
        motor_throat_follow.Set(0);
    }
}