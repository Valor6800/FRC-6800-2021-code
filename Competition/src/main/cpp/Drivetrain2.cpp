#include "Drivetrain2.h"

Drivetrain2::Drivetrain2() : kDriveKinematics{DriveConstants::kTrackwidth},
                                                            kSimpleMotorFeedforward{RamseteConstants::kS, RamseteConstants::kV, RamseteConstants::kA},
                                                            kTrajectoryConfigF{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
                                                            kTrajectoryConfigR{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
                                                            kDifferentialDriveVoltageConstraint{kSimpleMotorFeedforward, kDriveKinematics, 10_V},
                                                            leftDriveLead{DriveConstants::CAN_ID_LEFT_LEAD, rev::CANSparkMax::MotorType::kBrushless},
                                                            leftDriveFollowA{DriveConstants::CAN_ID_LEFT_FOLLOW_A, rev::CANSparkMax::MotorType::kBrushless},
                                                            leftDriveFollowB{DriveConstants::CAN_ID_LEFT_FOLLOW_B, rev::CANSparkMax::MotorType::kBrushless},
                                                            rightDriveLead{DriveConstants::CAN_ID_RIGHT_LEAD, rev::CANSparkMax::MotorType::kBrushless},
                                                            rightDriveFollowA{DriveConstants::CAN_ID_RIGHT_FOLLOW_A, rev::CANSparkMax::MotorType::kBrushless},
                                                            rightDriveFollowB{DriveConstants::CAN_ID_RIGHT_FOLLOW_B, rev::CANSparkMax::MotorType::kBrushless},
                                                            odometry{frc::Rotation2d(units::degree_t(getHeading()))} {
    
}

void Drivetrain2::setController(frc::XboxController* controller) {
    driverController = controller;
}

void Drivetrain2::init() {
    leftDriveLead.RestoreFactoryDefaults();
    leftDriveFollowA.RestoreFactoryDefaults();
    leftDriveFollowB.RestoreFactoryDefaults();
    rightDriveLead.RestoreFactoryDefaults();
    rightDriveFollowA.RestoreFactoryDefaults();
    rightDriveFollowB.RestoreFactoryDefaults();

    leftDriveLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftDriveFollowA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftDriveFollowB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightDriveLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightDriveFollowA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightDriveFollowB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    leftDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    rightDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);

    leftDriveFollowA.Follow(leftDriveLead);
    leftDriveFollowB.Follow(leftDriveLead);

    rightDriveFollowA.Follow(rightDriveLead);
    rightDriveFollowB.Follow(rightDriveLead);

    leftPIDController.SetP(DriveConstants::kP);
    leftPIDController.SetI(DriveConstants::kI);
    leftPIDController.SetD(DriveConstants::kD);
    leftPIDController.SetIZone(DriveConstants::kIz);
    leftPIDController.SetFF(DriveConstants::kFF);
    leftPIDController.SetOutputRange(DriveConstants::kMinOutput, DriveConstants::kMaxOutput);

    rightPIDController.SetP(DriveConstants::kP);
    rightPIDController.SetI(DriveConstants::kI);
    rightPIDController.SetD(DriveConstants::kD);
    rightPIDController.SetIZone(DriveConstants::kIz);
    rightPIDController.SetFF(DriveConstants::kFF);
    rightPIDController.SetOutputRange(DriveConstants::kMinOutput, DriveConstants::kMaxOutput);

    rightPIDController.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
    rightPIDController.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
    rightPIDController.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
    rightPIDController.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);
    
    leftPIDController.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
    leftPIDController.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
    leftPIDController.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
    leftPIDController.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);

    leftDriveLead.SetInverted(false);
    rightDriveLead.SetInverted(true);
}

void Drivetrain2::setDefaultState() {
    state.drivetrainState = DrivetrainState::DISABLED;

    resetState();
}

void Drivetrain2::assessInputs() {
    if (driverController->GetTriggerAxis(frc::GenericHID::kLeftHand) > 0.05 ||
        driverController->GetTriggerAxis(frc::GenericHID::kRightHand) > 0.05 ||
        std::abs(driverController->GetX(frc::GenericHID::kLeftHand)) > 0.05 ||
        driverController->GetYButton()) {
            state.drivetrainState = DrivetrainState::MANUAL;
            state.leftTrigger = driverController->GetTriggerAxis(frc::GenericHID::kLeftHand);
            state.rightTrigger = driverController->GetTriggerAxis(frc::GenericHID::kRightHand);
            state.leftJoystickX = driverController->GetX(frc::GenericHID::kLeftHand);
            state.Ybutton = driverController->GetYButton();
            state.rightBumper = driverController->GetBumper(frc::GenericHID::kRightHand);
    }
}

void Drivetrain2::assignOutputs() {
    // move to assessInputs()?
    odometry.Update(frc::Rotation2d(units::degree_t(getHeading())), getLeftDistance(), getRightDistance());
    
}

void Drivetrain2::resetState() {
    resetEncoders();
    resetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
    resetIMU();

    state.limelightState = LimelightConstants::STATE_ON; // Init as on so it immediately will turn off in the state machine
}

void Drivetrain2::resetEncoders() {
    leftCANEncoder.SetPosition(0);
    rightCANEncoder.SetPosition(0);
}

void Drivetrain2::resetOdometry(frc::Pose2d pose) {
    resetEncoders();
    odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(getHeading())));
}

void Drivetrain2::resetIMU() {
    imu.Reset();
}

units::meter_t Drivetrain2::getLeftDistance() {
    return leftCANEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

units::meter_t Drivetrain2::getRightDistance() {
    return rightCANEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

// check if the correct GetHeading was called last year
double Drivetrain2::getHeading() {
    return std::remainder(imu.GetAngle(), 360) * (RamseteConstants::kGyroReversed ? -1.0 : 1.0);
}

double Drivetrain2::getTurnRate() {
    return imu.GetRate() * (RamseteConstants::kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d Drivetrain2::getPose() {
    return odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drivetrain2::getWheelSpeeds() {
    return {units::meters_per_second_t(leftCANEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor),
            units::meters_per_second_t(rightCANEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor)};
}

void Drivetrain2::tankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts) {
    leftDriveLead.SetVoltage(leftVolts);
    rightDriveLead.SetVoltage(rightVolts);
}

