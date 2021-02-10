/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() : ValorSubsystem(),
                           leftDriveLead{DriveConstants::CAN_ID_LEFT_A, rev::CANSparkMax::MotorType::kBrushless},
                           leftDriveFollow{DriveConstants::CAN_ID_LEFT_B, rev::CANSparkMax::MotorType::kBrushless},
                           rightDriveLead{DriveConstants::CAN_ID_RIGHT_A, rev::CANSparkMax::MotorType::kBrushless},
                           rightDriveFollow{DriveConstants::CAN_ID_RIGHT_B, rev::CANSparkMax::MotorType::kBrushless},
                           kDriveKinematics{DriveConstants::kTrackwidth},
                           kSimpleMotorFeedforward{RamseteConstants::kS, RamseteConstants::kV, RamseteConstants::kA},
                           kTrajectoryConfigForward{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
                           kTrajectoryConfigReverse{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
                           kDifferentialDriveVoltageConstraint{kSimpleMotorFeedforward, kDriveKinematics, 10_V},
                           m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))},
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

    kTrajectoryConfigForward.SetKinematics(kDriveKinematics);
    kTrajectoryConfigForward.AddConstraint(kDifferentialDriveVoltageConstraint);
    kTrajectoryConfigForward.SetReversed(false);

    kTrajectoryConfigReverse.SetKinematics(kDriveKinematics);
    kTrajectoryConfigReverse.AddConstraint(kDifferentialDriveVoltageConstraint);
    kTrajectoryConfigReverse.SetReversed(true);

    imu.Calibrate();
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
    table->PutNumber("Heading", imu.GetAngle());
}

void Drivetrain::assignOutputs() {
    // update odemetry
    m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())), GetLeftDistance(), GetRightDistance());

    // arcade
    if (state.driveModeState == DriveModeState::ARCADE) {

        //assess inputs and determine target - move to seperate function
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
    ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
    ResetEncoders();
    ResetIMU();
}

void Drivetrain::ResetEncoders() {
    leftEncoder.SetPosition(0);
    rightEncoder.SetPosition(0);
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
    ResetEncoders();
    m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));
}

void Drivetrain::ResetIMU() {
    imu.Reset();
}

double Drivetrain::GetEncAvgDistance() {
    return ((leftEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor) + (rightEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor)) / 2.0;
}

units::meter_t Drivetrain::GetLeftDistance() {
    return leftEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

units::meter_t Drivetrain::GetRightDistance() {
    return rightEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

double Drivetrain::GetHeading() {
    return std::remainder(imu.GetAngle(), 360) * (RamseteConstants::kGyroReversed ? -1.0 : 1.0);
}

double Drivetrain::GetTurnRate() {
    return imu.GetRate() * (RamseteConstants::kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d Drivetrain::GetPose() { 
    return m_odometry.GetPose(); 
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
    return { units::meters_per_second_t(leftEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor),
             units::meters_per_second_t(rightEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor) };
}

void Drivetrain::TankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts) {
    leftDriveLead.SetVoltage(leftVolts);
    rightDriveLead.SetVoltage(rightVolts);
}

void Drivetrain::setPower(double power) {
    leftDriveLead.Set(power);
    rightDriveLead.Set(power);
}