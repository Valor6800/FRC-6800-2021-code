/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

// move driveModeState check out of assessiNputs?
// how to set state manual?

Drivetrain::Drivetrain() : ValorSubsystem(),
                           leftDriveLead{DriveConstants::CAN_ID_LEFT_A, rev::CANSparkMax::MotorType::kBrushless},
                           leftDriveFollow{DriveConstants::CAN_ID_LEFT_B, rev::CANSparkMax::MotorType::kBrushless},
                           rightDriveLead{DriveConstants::CAN_ID_RIGHT_A, rev::CANSparkMax::MotorType::kBrushless},
                           rightDriveFollow{DriveConstants::CAN_ID_RIGHT_B, rev::CANSparkMax::MotorType::kBrushless},
                           driverController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Drivetrain::init() {

    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("Drivetrain");

    leftDriveLead.RestoreFactoryDefaults();
    leftDriveFollow.RestoreFactoryDefaults();
    rightDriveLead.RestoreFactoryDefaults();
    rightDriveFollow.RestoreFactoryDefaults();

    leftDriveLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftDriveFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightDriveLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightDriveFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    leftDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    rightDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);

    leftDriveFollow.Follow(leftDriveLead);
    rightDriveFollow.Follow(rightDriveLead);

    leftDriveLead.SetInverted(true);
    rightDriveLead.SetInverted(false);
}

void Drivetrain::setController(frc::XboxController* controller) {
    driverController = controller;
}

void Drivetrain::setDefaultState() {
    state.tracking = false;
    state.driveModeState = DriveModeState::ROCKET_LEAGUE;

    resetState();
}

void Drivetrain::assessInputs() {
    if (!driverController) {
        return;
    }

    // driver inputs
    state.leftStickY = driverController->GetY(frc::GenericHID::kLeftHand);
    state.rightStickX = driverController->GetX(frc::GenericHID::kRightHand);
    
    state.leftTrigger = driverController->GetTriggerAxis(frc::GenericHID::kLeftHand);
    state.rightTrigger = driverController->GetTriggerAxis(frc::GenericHID::kRightHand);
    state.leftStickX = driverController->GetX(frc::GenericHID::kLeftHand);
    state.yButton = driverController->GetYButton();

    // drive mode
    if (driverController->GetBackButtonPressed()) {
        state.driveModeState = DriveModeState::ARCADE;
    }
    else if (driverController->GetStartButtonPressed()) {
        state.driveModeState = DriveModeState::ROCKET_LEAGUE;
    }

    // tracking
    state.tracking = driverController->GetBumper(frc::GenericHID::kRightHand);
}

void Drivetrain::analyzeDashboard() {
    table->PutNumber("Drive Mode", state.driveModeState);
    table->PutBoolean("Limelight Tracking", state.tracking);
}

void Drivetrain::assignOutputs() {
    
    // arcade
    if (state.driveModeState == DriveModeState::ARCADE) {
        //asses inputs and determine target - move to seperate function
        state.directionX = state.rightStickX / std::abs(state.rightStickX);

        state.straightTarget = -state.leftStickY;
        state.turnTarget = std::pow(state.rightStickX, 2) * state.directionX * DriveConstants::kArcTurnMultipler;

        if (std::abs(state.leftStickY) < DriveConstants::kDeadbandY) {
            state.straightTarget = 0;
        }

        if (std::abs(state.rightStickX) < DriveConstants::kDeadbandX) {
            state.turnTarget = 0;
        }

        state.currentLeftTarget = state.straightTarget + state.turnTarget;
        state.currentRightTarget = state.straightTarget - state.turnTarget;
    }
    // rocket league
    else {
        state.directionX = (state.leftStickX >= 0) ? -1 : 1;
        state.directionY = (state.leftTrigger - state.rightTrigger >= 0) ? 1 : -1;
        state.boostMultiplier = (state.yButton) ? DriveConstants::kBoost : DriveConstants::kNoBoost;

        state.straightTarget = -std::pow((state.leftTrigger - state.rightTrigger), 2) * state.directionY * state.boostMultiplier * DriveConstants::kDriveMultiplierY;
        state.turnTarget = -std::pow((state.leftStickX * DriveConstants::kDriveMultiplierX), 2) * state.directionX;

        if (std::abs(state.leftStickX) < DriveConstants::kDeadbandX) {
            state.turnTarget = 0;
        }

        if (std::abs(state.leftTrigger - state.rightTrigger) < DriveConstants::kDeadbandY) {
            state.straightTarget = 0;
        }

        state.currentLeftTarget = state.straightTarget + state.turnTarget;
        state.currentRightTarget = state.straightTarget - state.turnTarget;
    }

    int led_mode = limeTable->GetNumber("ledMode", LimelightConstants::LED_MODE_OFF);
    
    // Limelight Tracking
    if (state.tracking) {
        if (led_mode != LimelightConstants::LED_MODE_ON) { //prevent setting spam on limelight
            limeTable->PutNumber("ledMode", LimelightConstants::LED_MODE_ON);
            limeTable->PutNumber("camMode", LimelightConstants::TRACK_MODE_ON);
        }

        float tx = limeTable->GetNumber("tx", 0.0) * DriveConstants::limeLightKP ;

        leftDriveLead.Set(state.currentLeftTarget + tx);
        rightDriveLead.Set(state.currentRightTarget - tx);

    // Manual Control
    } else {
        if (led_mode != LimelightConstants::LED_MODE_OFF) { //prevent setting spam on limelight
            limeTable->PutNumber("ledMode", LimelightConstants::LED_MODE_OFF);
            limeTable->PutNumber("camMode", LimelightConstants::TRACK_MODE_OFF);
        }

        leftDriveLead.Set(state.currentLeftTarget);
        rightDriveLead.Set(state.currentRightTarget);
    }
}

void Drivetrain::resetState() {

}

void Drivetrain::setPower(double power) {
    leftDriveLead.Set(power);
    rightDriveLead.Set(power);
}