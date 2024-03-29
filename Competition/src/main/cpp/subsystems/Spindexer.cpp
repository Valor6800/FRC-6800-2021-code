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
    table->PutNumber("Drum High Speed Fender", SpindexerConstants::high_spd_drum_fender);
    table->PutNumber("Drum High Speed Initiation", SpindexerConstants::high_spd_drum_initiation);
    table->PutNumber("Drum High Speed Trench", SpindexerConstants::high_spd_drum_trench);
    table->PutNumber("Drum High Speed", SpindexerConstants::high_spd_drum);
    table->PutNumber("Throat Lead Speed", SpindexerConstants::default_throat_spd);
    table->PutNumber("Throat Follow Speed", SpindexerConstants::default_throat_spd);

    table->PutBoolean("Reverse", false);

    intakeTable = nt::NetworkTableInstance::GetDefault().GetTable("Intake");
    shooterTable = nt::NetworkTableInstance::GetDefault().GetTable("Shooter");

    motor_drum.RestoreFactoryDefaults();
    motor_throat.RestoreFactoryDefaults();
    motor_throat_follow.RestoreFactoryDefaults();

    motor_drum.SetClosedLoopRampRate(2);
    
    motor_drum.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_drum.SetInverted(false);
    
    motor_throat.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_throat.SetInverted(false);

    motor_throat.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    motor_throat_follow.Follow(rev::CANSparkMax::kFollowerDisabled, false);

    motor_throat_follow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_throat_follow.SetInverted(false);

    resetState();
}

void Spindexer::setController(frc::XboxController* controllerO, frc::XboxController* controllerD) {
    operatorController = controllerO;
    driverController = controllerD;
}

void Spindexer::setDefaultState() {
    state.drumState = DrumState::STOPPED;
    resetState();
}

void Spindexer::resetState() {
    state.current_cache_index = 0;
    for (int i = 0; i < SpindexerConstants::CACHE_SIZE; i++) {
        state.current_cache.push_back(0);
    }
}

void Spindexer::assessInputs() {
    if (!operatorController || !driverController) {
        return;
    }

    // Check if we are shooting. Run the drum faster if we are shooting
    if (std::abs(operatorController->GetTriggerAxis(frc::GenericHID::kLeftHand)) > SpindexerConstants::left_trigger_deadband) {
        state.drumState = DrumState::HIGH;
    } else if (driverController->GetStartButton()) {
        state.drumState = DrumState::REVERSE;
    } else if (state.drumState != DrumState::UNJAM) {
        state.drumState = DrumState::LOW;
    }

}

void Spindexer::analyzeDashboard() {
    table->PutNumber("Drum State", state.drumState);
    state.throat_lead_power = table->GetNumber("Throat Lead Speed", SpindexerConstants::default_throat_spd);
    state.throat_follow_power = table->GetNumber("Throat Follow Speed", SpindexerConstants::default_throat_spd);

    table->PutNumber("Drum Current", motor_drum.GetOutputCurrent());
    table->PutNumber("Throat Lead Current", motor_throat.GetOutputCurrent());
    table->PutNumber("Throat Follow Current", motor_throat_follow.GetOutputCurrent());

    table->PutNumber("Drum Inst. Current", state.instCurrent);

    // Check if the intake is deployed
    // If intake is not deployed, stop the drum
    bool intakeDeployState = intakeTable->GetBoolean("Deploy State", false);
    if (!intakeDeployState)
        state.drumState = DrumState::STOPPED;

    state.flywheelState = shooterTable->GetBoolean("FlywheelState", false);
    state.powerState = (Shooter::PowerState)shooterTable->GetNumber("PowerState", Shooter::PowerState::INITIATION);
}

void Spindexer::calcCurrent() {

    // Circular buffer. If index reaches end of vector, start at beginning and override prev values
    if (state.current_cache_index >= SpindexerConstants::CACHE_SIZE) {
        state.current_cache_index = 0;
    }
    // Set the current to the index, and increment the index
    state.current_cache.at(state.current_cache_index++) = motor_drum.GetOutputCurrent();

    // Calculate average current over the cache size, or circular buffer window
    double sum = 0;
    for (int i = 0; i < SpindexerConstants::CACHE_SIZE; i++) {
        sum += state.current_cache.at(i);
    }
    state.instCurrent = sum / SpindexerConstants::CACHE_SIZE;
}

void Spindexer::assignOutputs() {

    // Calculate instantaneous current
    calcCurrent();

    // Determine if we started an un-jamming session
    // Only run this logic on rising edge of state
    //   AKA, the FIRST time the instant current rises above the jam current
    if (state.instCurrent >= SpindexerConstants::JAM_CURRENT &&
        state.drumState == DrumState::LOW) {

        state.drumState = DrumState::UNJAM;
        state.initial_jam_position = drum_encoder.GetPosition();
    }

    // State LOW
    if (state.drumState == DrumState::LOW) {
        motor_drum.Set(table->GetNumber("Drum Low Speed", SpindexerConstants::default_drum_spd));
        motor_throat.Set(0);
        motor_throat_follow.Set(0);
    
    // State HIGH
    } else if (state.drumState == DrumState::HIGH && state.flywheelState) {

        if (state.powerState == Shooter::PowerState::FENDER) {
            motor_drum.Set(table->GetNumber("Drum High Speed Fender", SpindexerConstants::high_spd_drum_fender));
        } 
        else if (state.powerState == Shooter::PowerState::INITIATION)
        {
            motor_drum.Set(table->GetNumber("Drum High Speed Initiation", SpindexerConstants::high_spd_drum_initiation));
        }
        else if (state.powerState == Shooter::PowerState::TRENCH)
        {
            motor_drum.Set(table->GetNumber("Drum High Speed Trench", SpindexerConstants::high_spd_drum_trench));
        }
        else
        {
            motor_drum.Set(table->GetNumber("Drum High Speed", SpindexerConstants::high_spd_drum));
        }
            
        motor_throat.Set(state.throat_lead_power);
        motor_throat_follow.Set(state.throat_follow_power);
    
    // State UNJAM
    } else if (state.drumState == DrumState::UNJAM) {
        motor_drum.Set(-table->GetNumber("Drum Low Speed", SpindexerConstants::default_drum_spd));
        motor_throat.Set(0);
        motor_throat_follow.Set(0);

        // Reset the drum. Turn off unjamming if encoder value met
        if (drum_encoder.GetPosition() <= (state.initial_jam_position - SpindexerConstants::UMJAM_ROTATIONS)) {
            state.drumState = DrumState::LOW;
            resetState();
        }

    //State Reversed
    } else if (state.drumState == DrumState::REVERSE) {
        motor_drum.Set(-table->GetNumber("Drum Low Speed", SpindexerConstants::default_drum_spd));
        motor_throat.Set(-0.25);
        motor_throat_follow.Set(-0.25);
    
    // State STOPPED
    } else if (state.drumState == DrumState::STOPPED) {
        motor_drum.Set(0);
        motor_throat.Set(0);
        motor_throat_follow.Set(0);
    }
}