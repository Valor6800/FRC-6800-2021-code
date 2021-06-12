#include "subsystems/Shooter.h"

Shooter::Shooter() : ValorSubsystem(),
                     flywheel_follow{ShooterConstants::CAN_ID_FLYWHEEL_FOLLOW, rev::CANSparkMax::MotorType::kBrushless},
                     flywheel_lead{ShooterConstants::CAN_ID_FLYWHEEL_LEAD, rev::CANSparkMax::MotorType::kBrushless},
                     turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                     hood{ShooterConstants::SOLENOID_ID_SHOOTER},
                     operatorController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::init() {

    initTable("Shooter");
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight-turret");
    table->PutNumber("Manual Power", ShooterConstants::defaultManualPower);
    table->PutNumber("Flywheel Offset Power", 0);
    table->PutBoolean("Reset Turret", false);

    flywheel_follow.RestoreFactoryDefaults();
    flywheel_lead.RestoreFactoryDefaults();
    turret.RestoreFactoryDefaults();

    flywheel_follow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flywheel_lead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    turret.SetInverted(false);

    flywheel_lead.SetInverted(true);

    flywheel_lead.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    flywheel_follow.Follow(flywheel_lead, true);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitLeft);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitRight);

    // set PID coefficients
    pidController.SetP(ShooterConstants::shooterKP);
    pidController.SetI(ShooterConstants::shooterKI);
    pidController.SetD(ShooterConstants::shooterKD);
    pidController.SetIZone(ShooterConstants::shooterKFF);
    pidController.SetFF(ShooterConstants::shooterKIZ);
    pidController.SetOutputRange(0, 1);

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
    state.yButton = operatorController->GetYButton();
    state.startButton = operatorController->GetStartButton();
    state.backButton = operatorController->GetBackButton();
    state.dpad = operatorController->GetPOV();

    // test for default value for shuffleboard entry not existing
    state.manualPow = table->GetNumber("Manual Power", ShooterConstants::defaultManualPower);
    state.flywheelOffsetPow = table->GetNumber("Flywheel Offset Power", 0);

    // Turret
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::MANUAL_TURRET;
    } else if (state.yButton) {
        state.turretState = TurretState::HOME;
        state.turretSetpoint = ShooterConstants::homePosition;
    } else if (state.xButton) {
        state.turretState = TurretState::HOME;
        state.turretSetpoint = ShooterConstants::homeFrontPosition;;
    } else if (state.rightBumper) {
        state.turretState = TurretState::TRACK;
        // state.powerState = PowerState::DYNAMIC;
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

    table->PutNumber("FollowCurrent", flywheel_follow.GetOutputCurrent());
    table->PutNumber("LeadCurrent", flywheel_lead.GetOutputCurrent());

    table->PutNumber("ShooterRPM", flywheel_encoder.GetVelocity());

    if (table->GetBoolean("Reset Turret", false)) {
        resetEncoder();
    }
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

        // Minimum power deadband
        if (std::abs(state.turretTarget) < ShooterConstants::pDeadband) {
            state.turretTarget = 0;
        }
        // Stop deadband
        else if (std::abs(state.turretTarget) < ShooterConstants::pSoftDeadband) {
            int direction = 1;
            if (state.turretTarget < 0) direction = -1;
            state.turretTarget = ShooterConstants::pSoftDeadband * direction;
        }

    // HOME
    } else if (state.turretState == TurretState::HOME) {
        state.error = turretEncoder.GetPosition() - state.turretSetpoint;
        state.turretTarget = std::pow(ShooterConstants::turretKP * -state.error, 3) * ShooterConstants::turretKQ;

    // TRACK
    } else if (state.turretState == TurretState::TRACK) {
        float tx = limeTable->GetNumber("tx", 0.0);
        float tv = limeTable->GetNumber("tv" , 0.0);
        state.turretTarget = tv * -tx * ShooterConstants::limelightTurnKp;
    }

    table->PutNumber("TurretTarget", state.turretTarget);

     // turret output
    turret.Set(state.turretTarget);

    // Flywheel *********************************

    // FENDER
    if (state.powerState == PowerState::FENDER) {
        state.flywheelTarget = ShooterConstants::fenderPower;
        state.hoodTarget = true;
    
    // INITIATION
    } else if (state.powerState == PowerState::INITIATION) {
        state.flywheelTarget = ShooterConstants::initiationPower;
        state.hoodTarget = false;
    
    // TRENCH
    } else if (state.powerState == PowerState::TRENCH) {
        state.flywheelTarget = ShooterConstants::trenchPower;
        state.hoodTarget = false;

    // MANUAL
    } else if (state.powerState == PowerState::MANUAL_POWER) {
        state.flywheelTarget = state.manualPow;
        state.hoodTarget = false;

    // LIMELIGHT
    } else if (state.powerState == PowerState::DYNAMIC) {
        state.flywheelTarget = ShooterConstants::limelightDistKp * state.limelightDistance;
        state.hoodTarget = false;
    }

    // hood output
    hood.Set(state.hoodTarget);

    state.flywheelTarget = state.flywheelTarget > 0 ? state.flywheelTarget : 0;

    // flywheel output
    if (state.shooterState) {
        pidController.SetReference(state.flywheelTarget + state.flywheelOffsetPow, rev::ControlType::kVelocity);

    } else {
        flywheel_lead.Set(0);
        pidController.SetIAccum(0);
    }
}

void Shooter::resetEncoder(){
    turretEncoder.SetPosition(ShooterConstants::homePosition); //reset encoder home position
}

void Shooter::resetState() {
    resetEncoder();
}