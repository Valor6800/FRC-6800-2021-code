/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drivetrain.h"

Drivetrain::Drivetrain() : ValorSubsystem(), 
                           kDriveKinematics{DriveConstants::kTrackwidth},
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
                           odometry{frc::Rotation2d(units::degree_t(getHeading()))},
                           driverController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

void Drivetrain::setController(frc::XboxController* controller) {
    driverController = controller;
}

void Drivetrain::init() {
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
    
    leftPIDController.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
    leftPIDController.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
    leftPIDController.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
    leftPIDController.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);

    rightPIDController.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
    rightPIDController.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
    rightPIDController.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
    rightPIDController.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);

    leftDriveLead.SetInverted(false);
    rightDriveLead.SetInverted(true);

    kTrajectoryConfigF.SetKinematics(kDriveKinematics);
    kTrajectoryConfigF.AddConstraint(kDifferentialDriveVoltageConstraint);
    kTrajectoryConfigF.SetReversed(false);

    kTrajectoryConfigR.SetKinematics(kDriveKinematics);
    kTrajectoryConfigR.AddConstraint(kDifferentialDriveVoltageConstraint);
    kTrajectoryConfigR.SetReversed(true);

    imu.Calibrate();
}

void Drivetrain::setDefaultState() {
    state.drivetrainState = DrivetrainState::DISABLED;

    resetState();
}

void Drivetrain::assessInputs() {
    if (!driverController) {
        return;
    }

    if (driverController->GetTriggerAxis(frc::GenericHID::kLeftHand) > DriveConstants::kDeadbandTrigger ||
        driverController->GetTriggerAxis(frc::GenericHID::kRightHand) > DriveConstants::kDeadbandTrigger ||
        std::abs(driverController->GetX(frc::GenericHID::kLeftHand)) > DriveConstants::kDeadbandX ||
        driverController->GetYButton()) {
            state.drivetrainState = DrivetrainState::MANUAL;

            frc::SmartDashboard::PutNumber("Left Trigger", driverController->GetTriggerAxis(frc::GenericHID::kLeftHand));
            frc::SmartDashboard::PutNumber("Right Trigger", driverController->GetTriggerAxis(frc::GenericHID::kRightHand));
            frc::SmartDashboard::PutNumber("Left Stick X", driverController->GetX(frc::GenericHID::kLeftHand));
            frc::SmartDashboard::PutBoolean("Right Bumper", driverController->GetBumper(frc::GenericHID::kRightHand));

            // inputs
            state.leftTrigger = driverController->GetTriggerAxis(frc::GenericHID::kLeftHand);
            state.rightTrigger = driverController->GetTriggerAxis(frc::GenericHID::kRightHand);
            state.leftJoystickX = driverController->GetX(frc::GenericHID::kLeftHand);
            state.Ybutton = driverController->GetYButton();
            state.rightBumper = driverController->GetBumper(frc::GenericHID::kRightHand);

            state.directionX = (state.leftJoystickX >= 0) ? 1 : -1;
            state.directionY = (state.rightTrigger - state.leftTrigger >= 0) ? 1 : -1;
            
            state.boostMultiplier = (state.rightBumper) ? DriveConstants::kBoost : DriveConstants::kNoBoost;

            frc::SmartDashboard::PutNumber("Boost Multiplier", state.boostMultiplier);

            // target references for pid controller
            state.straightTarget = -std::pow((state.rightTrigger - state.leftTrigger), 2) * state.directionY * state.boostMultiplier * DriveConstants::kDriveMultiplierY * DriveConstants::MAX_RPM;
            state.turnTarget = -std::pow((state.leftJoystickX * DriveConstants::kDriveMultiplierX), 2) * state.directionX * DriveConstants::MAX_RPM;

            // x axis deadband check
            if (std::abs(state.leftJoystickX) < DriveConstants::kDeadbandX) {
                state.turnTarget = 0;
            }

            // y axis deadband check
            if (std::abs(state.rightTrigger - state.leftTrigger) < DriveConstants::kDeadbandY) {
                state.straightTarget = 0;
            }

            // drive straightening
            if (std::abs(state.leftJoystickX) < DriveConstants::kDeadbandX) {
                if (state.straightTarget == 0) {
                    state.turnTarget = 0;
                }
                else {
                    state.turnTarget = (leftCANEncoder.GetVelocity() - rightCANEncoder.GetVelocity()) * DriveConstants::kDriveOffset;
                }
            }

            // limelight
            std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
            float tx = table->GetNumber("tx", 0.0);
            float tv = table->GetNumber("tv" , 0.0);

            if (state.Ybutton) {
                table->PutNumber("ledMode", LimelightConstants::LED_MODE_ON);
                table->PutNumber("camMode", LimelightConstants::TRACK_MODE_ON);
                state.limelightState = LimelightConstants::STATE_ON;

                if (tv == 1) {
                    state.turnTarget = tx * DriveConstants::kP * DriveConstants::MAX_RPM;
                }
                else {
                    table->PutNumber("ledMode", LimelightConstants::LED_MODE_OFF);
                    table->PutNumber("camMode", LimelightConstants::TRACK_MODE_OFF);
                    state.limelightState = LimelightConstants::STATE_OFF;
                }
            }

            state.currentLeftTarget = state.straightTarget - state.turnTarget;
            state.currentRightTarget = state.straightTarget + state.turnTarget;
    }
    else {
        state.drivetrainState = DrivetrainState::DISABLED;
    }
}

void Drivetrain::assignOutputs() {
    odometry.Update(frc::Rotation2d(units::degree_t(getHeading())), getLeftDistance(), getRightDistance());

    state.drivetrainState == DrivetrainState::MANUAL ? frc::SmartDashboard::PutString("State", "Manual") : frc::SmartDashboard::PutString("State", "Disabled");

    if (state.drivetrainState == DrivetrainState::MANUAL) {
        frc::SmartDashboard::PutNumber("Left PID Reference", state.currentLeftTarget);
        frc::SmartDashboard::PutNumber("Right PID Reference", state.currentRightTarget);

        leftPIDController.SetReference(state.currentLeftTarget, rev::ControlType::kVelocity);
        rightPIDController.SetReference(state.currentRightTarget, rev::ControlType::kVelocity);
        
        frc::SmartDashboard::PutNumber("Left PID Reference", state.currentLeftTarget);
        frc::SmartDashboard::PutNumber("Right PID Reference", state.currentRightTarget);
    }
    else if (state.drivetrainState == DrivetrainState::AUTO) {

    }
    else {
        setPower(0);
    }
}

void Drivetrain::resetState() {
    resetEncoders();
    resetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
    resetIMU();

    state.limelightState = LimelightConstants::STATE_ON; // Init as on so it immediately will turn off in the state machine
}

void Drivetrain::setPower(double power) {
    leftDriveLead.Set(power);
    rightDriveLead.Set(power);
}

void Drivetrain::resetEncoders() {
    leftCANEncoder.SetPosition(0);
    rightCANEncoder.SetPosition(0);
}

void Drivetrain::resetOdometry(frc::Pose2d pose) {
    resetEncoders();
    odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(getHeading())));
}

void Drivetrain::resetIMU() {
    imu.Reset();
}

units::meter_t Drivetrain::getLeftDistance() {
    return leftCANEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

units::meter_t Drivetrain::getRightDistance() {
    return rightCANEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

double Drivetrain::getHeading() {
    return std::remainder(imu.GetAngle(), 360) * (RamseteConstants::kGyroReversed ? -1.0 : 1.0);
}

double Drivetrain::getTurnRate() {
    return imu.GetRate() * (RamseteConstants::kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d Drivetrain::getPose() {
    return odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drivetrain::getWheelSpeeds() {
    return {units::meters_per_second_t(leftCANEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor),
            units::meters_per_second_t(rightCANEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor)};
}

void Drivetrain::tankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts) {
    leftDriveLead.SetVoltage(leftVolts);
    rightDriveLead.SetVoltage(rightVolts);
}

