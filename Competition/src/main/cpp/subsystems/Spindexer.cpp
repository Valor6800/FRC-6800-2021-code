#include "subsystems/Spindexer.h"

Spindexer::Spindexer() : ValorSubsystem(),
                        motor_drum{SpindexerConstants::CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        motor_throat{SpindexerConstants::CAN_ID_THROAT, rev::CANSparkMax::MotorType::kBrushless},
                        motor_throat_follow{SpindexerConstants::CAN_ID_THROAT_FOLLOW, rev::CANSparkMax::MotorType::kBrushless} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Spindexer::init() {

    initTable("Spindexer");
    table->PutNumber("Drum Low Speed", SpindexerConstants::default_drum_spd);
    table->PutNumber("Drum High Speed", SpindexerConstants::high_spd_drum);
    table->PutNumber("Throat Speed", SpindexerConstants::default_throat_spd);

    motor_drum.RestoreFactoryDefaults();
    motor_throat.RestoreFactoryDefaults();
    motor_throat_follow.RestoreFactoryDefaults();
    
    motor_drum.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_drum.SetInverted(false);
    
    motor_throat.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    motor_throat.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    motor_throat_follow.Follow(rev::CANSparkMax::kFollowerDisabled, false);

    motor_throat_follow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_throat_follow.SetInverted(true);
}

void Spindexer::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Spindexer::setDefaultState() {
    state.spinState = false;
    resetState();
}

void Spindexer::resetState() {

}

void Spindexer::assessInputs() {
    if (!operatorController) {
        return;
    }

    state.spinState = std::abs(operatorController->GetTriggerAxis(frc::GenericHID::kLeftHand)) > SpindexerConstants::left_trigger_deadband;

}

void Spindexer::analyzeDashboard() {
    table->PutBoolean("Drum State", state.spinState);
    state.throat_power = table->GetNumber("Throat Speed", SpindexerConstants::default_throat_spd);
}

void Spindexer::assignOutputs() {
    if (state.spinState) {        
        motor_drum.Set(table->GetNumber("Drum Low Speed", SpindexerConstants::high_spd_drum));       
        motor_throat.Set(state.throat_power * -1); //reverse direction
        motor_throat_follow.Set(state.throat_power);
    } else {
        motor_drum.Set(table->GetNumber("Drum Low Speed", SpindexerConstants::default_drum_spd));
        motor_throat.Set(0);
        motor_throat_follow.Set(0);
    }
}