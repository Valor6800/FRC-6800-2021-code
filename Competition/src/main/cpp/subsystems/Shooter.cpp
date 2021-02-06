#include "subsystems/Shooter.h"

Shooter::Shooter() : ValorSubsystem(),
                     flywheelA{ShooterConstants::CAN_ID_FLYWHEEL_A, rev::CANSparkMax::MotorType::kBrushless},
                     flywheelB{ShooterConstants::CAN_ID_FLYWHEEL_B, rev::CANSparkMax::MotorType::kBrushless},
                     turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                     hood{ShooterConstants::SOLENOID_ID_SHOOTER, ShooterConstants::SOLENOID_ID_SHOOTER},
                     operatorController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::init() {

    initTable("Shooter");
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight2");
    table->PutNumber("Manual Power", ShooterConstants::defaultManualPower);
    table->PutNumber("Flywheel Offset Power", 0);

    flywheelA.RestoreFactoryDefaults();
    flywheelB.RestoreFactoryDefaults();
    turret.RestoreFactoryDefaults();

    flywheelA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flywheelB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    flywheelA.SetInverted(false);
    flywheelB.SetInverted(true);
    turret.SetInverted(false);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitLeft);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitRight);

    resetState(); //reset shooter/encoder state
}

void Shooter::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Shooter::setDefaultState() {
    state.shooterState = false;
    state.powerState = PowerState::INITIATION;
    state.turretState = TurretState::DISABLED_TURRET;
}

void Shooter::assessInputs() {
    if (!operatorController) {
        return;
    }

    // driver inputs
    state.leftStickX = -operatorController->GetX(frc::GenericHID::kLeftHand);
    state.rightBumper = operatorController->GetBumper(frc::GenericHID::kRightHand);
    state.xButton = operatorController->GetXButton();
    state.startButton = operatorController->GetStartButton();
    state.backButton = operatorController->GetBackButton();
    state.dpad = operatorController->GetPOV();

    // test for default value for shuffleboard entry not existing
    state.manualPow = table->GetNumber("Manual Power", ShooterConstants::defaultManualPower);
    state.flywheelOffsetPow = table->GetNumber("Flywheel Offset Power", 0);

    // Turret
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::MANUAL_TURRET;
    } else if (state.xButton) {
        state.turretState = TurretState::HOME;
    } else if (state.rightBumper) {
        state.turretState = TurretState::TRACK;
        state.powerState = PowerState::DYNAMIC;
    } else {
        state.turretState = TurretState::DISABLED_TURRET;
    }

    // Shooter
    if (state.startButton) {
        state.shooterState = true;
    } else if (state.backButton) {
        state.shooterState = false;
    }

    // Power
    if (state.dpad == ShooterConstants::dpadDown) {
        state.powerState = PowerState::TRENCH;
    } else if (state.dpad == ShooterConstants::dpadRight) {
        state.powerState = PowerState::INITIATION;
    } else if (state.dpad == ShooterConstants::dpadUp) {
        state.powerState = PowerState::FENDER;
    } else if (state.dpad == ShooterConstants::dpadLeft) {
        state.powerState = PowerState::MANUAL_POWER;
    }
}

void Shooter::limelightTrack(bool track) {
    limeTable->PutNumber("ledMode", track ? LimelightConstants::LED_MODE_ON : LimelightConstants::LED_MODE_OFF);
    limeTable->PutNumber("camMode", track ? LimelightConstants::TRACK_MODE_ON : LimelightConstants::TRACK_MODE_OFF);
}

void Shooter::analyzeDashboard() {
    limelightTrack(state.turretState == TurretState::TRACK);
    table->PutNumber("TurretState", state.turretState);
    table->PutBoolean("FlywheelState", state.shooterState);
    table->PutNumber("PowerState", state.powerState);
    table->PutBoolean("HoodState", state.hoodTarget);
    table->PutNumber("TurretEncoder", turretEncoder.GetPosition());
    table->PutNumber("TurretEncoderVelocity", turretEncoder.GetVelocity());
}

void Shooter::assignOutputs() {
    
    // Turret ******************************************************
    state.turretTarget = 0;

    // DISABLED
    if (state.turretState == TurretState::DISABLED_TURRET) {
        state.turretTarget = 0;

    // MANUAL
    } else if (state.turretState == TurretState::MANUAL_TURRET) {
        int sign = state.leftStickX >= 0 ? 1 : -1;
        state.turretTarget = sign * std::pow(state.leftStickX, 2);

    // HOME
    } else if (state.turretState == TurretState::HOME) {
        state.error = turretEncoder.GetPosition();

        if (std::abs(state.error) > ShooterConstants::pDeadband) {
            state.turretTarget = ShooterConstants::turretKP * -state.error;
        } else {
            state.turretTarget = 0;
        }

    // TRACK
    } else if (state.turretState == TurretState::TRACK) {
        float ty = limeTable->GetNumber("ty", 0.0);
        float tv = limeTable->GetNumber("tv" , 0.0);
        state.turretTarget = tv * ty * ShooterConstants::limelightTurnKp;
    }

     // turret output
    turret.Set(state.turretTarget);

    // Flywheel *********************************

    // DISABLED
    if (!state.shooterState) {
        state.flywheelTarget = 0; 
        state.hoodTarget = true;

    // FENDER
    } else if (state.powerState == PowerState::FENDER) {
        state.flywheelTarget = ShooterConstants::fenderPower;
        state.hoodTarget = false;
    
    // INITIATION
    } else if (state.powerState == PowerState::INITIATION) {
        state.flywheelTarget = ShooterConstants::initiationPower;
        state.hoodTarget = true;
    
    // TRENCH
    } else if (state.powerState == PowerState::TRENCH) {
        state.flywheelTarget = ShooterConstants::trenchPower;
        state.hoodTarget = true;

    // MANUAL
    } else if (state.powerState == PowerState::MANUAL_POWER) {
        state.flywheelTarget = state.manualPow;
        state.hoodTarget = true;

    // LIMELIGHT
    } else if (state.powerState == PowerState::DYNAMIC) {
        state.flywheelTarget = ShooterConstants::limelightDistKp * state.limelightDistance;
        state.hoodTarget = true;
    }

    // hood output
    hood.Set(state.hoodTarget);

    // flywheel output
    if (state.shooterState) {
        flywheelA.Set(state.flywheelTarget + state.flywheelOffsetPow);
        flywheelB.Set(state.flywheelTarget + state.flywheelOffsetPow);

    } else {
        flywheelA.Set(0);
        flywheelB.Set(0);
    }
}

void Shooter::resetEncoder(){
    turretEncoder.SetPosition(ShooterConstants::homePosition); //reset encoder home position
}

void Shooter::resetState() {
    resetEncoder();
}