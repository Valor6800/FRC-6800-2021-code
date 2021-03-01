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
    table->PutNumber("Throat Lead Speed", SpindexerConstants::default_throat_spd);
    table->PutNumber("Throat Follow Speed", SpindexerConstants::default_throat_spd);

    for (int i = 0; i < SpindexerConstants::CACHE_SIZE; i++) {
        state.current_cache.push_back(0);
    }
    state.direction = 1;
    state.current_cache_index = 0;

    intakeTable = nt::NetworkTableInstance::GetDefault().GetTable("Intake");

    motor_drum.RestoreFactoryDefaults();
    motor_throat.RestoreFactoryDefaults();
    motor_throat_follow.RestoreFactoryDefaults();
    
    motor_drum.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_drum.SetInverted(false);
    
    motor_throat.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_throat.SetInverted(false);

    motor_throat.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    motor_throat_follow.Follow(rev::CANSparkMax::kFollowerDisabled, false);

    motor_throat_follow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_throat_follow.SetInverted(false);
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
    state.throat_lead_power = table->GetNumber("Throat Lead Speed", SpindexerConstants::default_throat_spd);
    state.throat_follow_power = table->GetNumber("Throat Follow Speed", SpindexerConstants::default_throat_spd);

    table->PutNumber("Drum Current", motor_drum.GetOutputCurrent());
    table->PutNumber("Throat Lead Current", motor_throat.GetOutputCurrent());
    table->PutNumber("Throat Follow Current", motor_throat_follow.GetOutputCurrent());

    state.deployState = intakeTable->GetBoolean("Deploy State", false);
}

void Spindexer::assignOutputs() {

    if (state.current_cache_index >= SpindexerConstants::CACHE_SIZE) {
        state.current_cache_index = 0;
    }
    state.current_cache.at(state.current_cache_index++) = motor_drum.GetOutputCurrent();

    double sum = 0;
    for (int i = 0; i < SpindexerConstants::CACHE_SIZE; i++) {
        sum += state.current_cache.at(i);
    }
    double inst_current = sum / SpindexerConstants::CACHE_SIZE;
    if (inst_current > 9) {
        state.direction = -1;
    }

    // @TODO we need a 200ms delay
    if (state.deployState) {
        if (state.spinState) {        
            motor_drum.Set(table->GetNumber("Drum High Speed", SpindexerConstants::high_spd_drum));       
            motor_throat.Set(state.throat_lead_power); //reverse direction
            motor_throat_follow.Set(state.throat_follow_power);
        } else {
            motor_drum.Set(table->GetNumber("Drum Low Speed", SpindexerConstants::default_drum_spd) * state.direction);
            motor_throat.Set(0);
            motor_throat_follow.Set(0);
        }
    } else {
        motor_drum.Set(0);
        motor_throat.Set(0);
        motor_throat_follow.Set(0);
    }
}