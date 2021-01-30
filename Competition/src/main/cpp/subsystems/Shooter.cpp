#include "subsystems/Shooter.h"

Shooter::Shooter() : ValorSubsystem(),
                     flywheelA{ShooterConstants::CAN_ID_FLYWHEEL_A, rev::CANSparkMax::MotorType::kBrushless},
                     flywheelB{ShooterConstants::CAN_ID_FLYWHEEL_B, rev::CANSparkMax::MotorType::kBrushless},
                     turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                     hood{ShooterConstants::SOLENOID_ID_SHOOTER, ShooterConstants::SOLENOID_ID_SHOOTER},
                     operatorController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void Shooter::init() {
    flywheelA.RestoreFactoryDefaults();
    flywheelB.RestoreFactoryDefaults();
    turret.RestoreFactoryDefaults();

    flywheelA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flywheelB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // turret on brake?
    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // test
    flywheelA.SetInverted(false);
    flywheelB.SetInverted(true);
    turret.SetInverted(false);

    manualPower = frc::Shuffleboard::GetTab("Configuration").Add("Manual Power", 1).WithWidget("Text View").GetEntry();
    flywheelOffsetPower = frc::Shuffleboard::GetTab("Configuration").Add("Flywheel Offset Power", 1).WithWidget("Text View").GetEntry();
}

void Shooter::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Shooter::setDefaultState() {
    state.shooterState = ShooterState::DISABLED_SHOOTER;
    state.powerState = PowerState::INITIATION;
    state.turretState = TurretState::DISABLED_TURRET;
}

void Shooter::assessInputs() {
    if (!operatorController) {
        return;
    }

    // driver inputs
    state.leftStickX = operatorController->GetX(frc::GenericHID::kLeftHand);
    state.rightBumper = operatorController->GetBumper(frc::GenericHID::kRightHand);
    state.xButton = operatorController->GetXButton();
    state.aButton = operatorController->GetAButton();
    state.startButton = operatorController->GetStartButton();
    state.backButton = operatorController->GetBackButton();
    state.dpad = operatorController->GetPOV();
    // test for default value for shuffleboard entry not existing
    state.manualPow = manualPower.GetDouble(0);
    state.flywheelOffsetPow = flywheelOffsetPower.GetDouble(0);

    // turret state
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::MANUAL_TURRET;
    }
    else if (state.xButton) {
        state.turretState = TurretState::HOME;
    }
    else if (state.rightBumper) {
        state.turretState = TurretState::TRACK;
        state.powerState = PowerState::DYNAMIC;
    }
    else {
        state.turretState = TurretState::HOLD;
    }

    // shooter state
    if (state.startButton) {
        state.shooterState = ShooterState::ON;
    }
    else if (state.backButton) {
        state.shooterState = ShooterState::OFF;
    }

    // flywheel power state
    if (state.dpad == ShooterConstants::dpadDown) {
        state.powerState = PowerState::FENDER;
    }
    else if (state.dpad == ShooterConstants::dpadRight) {
        state.powerState = PowerState::INITIATION;
    }
    else if (state.dpad == ShooterConstants::dpadUp) {
        state.powerState = PowerState::TRENCH;
    }
    else if (state.dpad == ShooterConstants::dpadLeft) {
        state.powerState = PowerState::MANUAL_POWER;
    }
}

void Shooter::assignOutputs() {
    // turret target
    if (state.turretState == TurretState::DISABLED_TURRET || state.turretState == TurretState::HOLD) {
        state.turretTarget = 0;
    }
    else if (state.turretState == TurretState::MANUAL_TURRET) {
        // limit deadband coerce scaling?
        state.turretTarget = state.leftStickX;
    }
    else if (state.turretState == TurretState::HOME) {
        // error with encoder?
        state.error = 0;
        state.pGain = ShooterConstants::turretKP * state.error;

        if (std::abs(state.error) > ShooterConstants::pDeadband) {
            state.turretTarget = state.pGain * state.error;
        }
        else {
            state.turretTarget = 0;
        }
    }
    else if (state.turretState == TurretState::TRACK) {
        std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        float ty = table->GetNumber("ty", 0.0);
        float tv = table->GetNumber("tv" , 0.0);

        table->PutNumber("ledMode", LimelightConstants::LED_MODE_ON);
        table->PutNumber("camMode", LimelightConstants::TRACK_MODE_ON);
            
        if (tv == 1) {
            state.turretTarget = ty;
        }
    
        table->PutNumber("ledMode", LimelightConstants::LED_MODE_OFF);
        table->PutNumber("camMode", LimelightConstants::TRACK_MODE_OFF);
    }
    else {
        state.turretTarget = 0;
    }

    // flywheel target
    if (state.powerState == PowerState::DISABLED_POWER || state.shooterState == ShooterState::DISABLED_SHOOTER) {
        state.flywheelTarget = 0;
    }
    else if (state.powerState == PowerState::FENDER) {
        state.flywheelTarget = ShooterConstants::fenderPower;
        state.hoodTarget = false;
    }
    else if (state.powerState == PowerState::INITIATION) {
        state.flywheelTarget = ShooterConstants::initiationPower;
        state.hoodTarget = true;
    }
    else if (state.powerState == PowerState::TRENCH) {
        state.flywheelTarget = ShooterConstants::trenchPower;
        state.hoodTarget = true;
    }
    else if (state.powerState == PowerState::MANUAL_POWER) {
        state.flywheelTarget = state.manualPow;
        state.hoodTarget = true;
    }
    else if (state.powerState == PowerState::DYNAMIC) {
        state.flywheelTarget = ShooterConstants::limelightKP * state.limelightDistance;
        state.hoodTarget = true;
    }

    // turret output
    turret.Set(state.turretTarget);

    // hood output
    hood.Set(state.hoodTarget);

    // flywheel output
    if (state.shooterState == ShooterState::ON) {
        flywheelA.Set(state.flywheelTarget + state.flywheelOffsetPow);
        flywheelB.Set(state.flywheelTarget + state.flywheelOffsetPow);
    }
    else {
        flywheelA.Set(0);
        flywheelB.Set(0);
    }
}

void Shooter::resetState() {

}