/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drivetrain.h"

Drivetrain::Drivetrain() : boostMultiplier{DriveConstants::kNoBoost},
                           kDriveKinematics{DriveConstants::kTrackwidth},
                           kSimpleMotorFeedforward{RamseteConstants::kS, RamseteConstants::kV, RamseteConstants::kA},
                           kTrajectoryConfigF{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
                           kTrajectoryConfigR{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
                           kDifferentialDriveVoltageConstraint{kSimpleMotorFeedforward, kDriveKinematics, 10_V},
                           m_leftDriveLead{DriveConstants::CAN_ID_LEFT_LEAD, rev::CANSparkMax::MotorType::kBrushless},
                           m_leftDriveFollowA{DriveConstants::CAN_ID_LEFT_FOLLOW_A, rev::CANSparkMax::MotorType::kBrushless},
                           m_leftDriveFollowB{DriveConstants::CAN_ID_LEFT_FOLLOW_B, rev::CANSparkMax::MotorType::kBrushless},
                           m_rightDriveLead{DriveConstants::CAN_ID_RIGHT_LEAD, rev::CANSparkMax::MotorType::kBrushless},
                           m_rightDriveFollowA{DriveConstants::CAN_ID_RIGHT_FOLLOW_A, rev::CANSparkMax::MotorType::kBrushless},
                           m_rightDriveFollowB{DriveConstants::CAN_ID_RIGHT_FOLLOW_B, rev::CANSparkMax::MotorType::kBrushless},
                           m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))},
                           limelight_state{LimelightConstants::STATE_ON} // Init as on so it immediately will turn off in the state machine
{
  kTrajectoryConfigF.SetKinematics(kDriveKinematics);
  kTrajectoryConfigF.AddConstraint(kDifferentialDriveVoltageConstraint);
  kTrajectoryConfigF.SetReversed(false);

  kTrajectoryConfigR.SetKinematics(kDriveKinematics);
  kTrajectoryConfigR.AddConstraint(kDifferentialDriveVoltageConstraint);
  kTrajectoryConfigR.SetReversed(true);

  imu.Calibrate();

  InitDrives(rev::CANSparkMax::IdleMode::kCoast);
}

Drivetrain& Drivetrain::GetInstance() {
  static Drivetrain instance;
  return instance;
}

void Drivetrain::Periodic() {
  m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())), GetLeftDistance(), GetRightDistance());
}

void Drivetrain::InitDrives(rev::CANSparkMax::IdleMode idleMode) {
  m_leftDriveLead.RestoreFactoryDefaults();
  m_leftDriveFollowA.RestoreFactoryDefaults();
  m_leftDriveFollowB.RestoreFactoryDefaults();
  m_rightDriveLead.RestoreFactoryDefaults();
  m_rightDriveFollowA.RestoreFactoryDefaults();
  m_rightDriveFollowB.RestoreFactoryDefaults();

  m_leftDriveLead.SetIdleMode(idleMode);
  m_leftDriveFollowA.SetIdleMode(idleMode);
  m_leftDriveFollowB.SetIdleMode(idleMode);
  m_rightDriveLead.SetIdleMode(idleMode);
  m_rightDriveFollowA.SetIdleMode(idleMode);
  m_rightDriveFollowB.SetIdleMode(idleMode);

  // make sure lead motors are not following anything
  m_leftDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);
  m_rightDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);

  // set left follow motors to follow lead
  m_leftDriveFollowA.Follow(m_leftDriveLead);
  m_leftDriveFollowB.Follow(m_leftDriveLead);

  // set right follow motors to follow lead
  m_rightDriveFollowA.Follow(m_rightDriveLead);
  m_rightDriveFollowB.Follow(m_rightDriveLead);

  m_leftPIDController.SetP(DriveConstants::kP);
  m_leftPIDController.SetI(DriveConstants::kI);
  m_leftPIDController.SetD(DriveConstants::kD);
  m_leftPIDController.SetIZone(DriveConstants::kIz);
  m_leftPIDController.SetFF(DriveConstants::kFF);
  m_leftPIDController.SetOutputRange(DriveConstants::kMinOutput, DriveConstants::kMaxOutput);

  m_rightPIDController.SetP(DriveConstants::kP);
  m_rightPIDController.SetI(DriveConstants::kI);
  m_rightPIDController.SetD(DriveConstants::kD);
  m_rightPIDController.SetIZone(DriveConstants::kIz);
  m_rightPIDController.SetFF(DriveConstants::kFF);
  m_rightPIDController.SetOutputRange(DriveConstants::kMinOutput, DriveConstants::kMaxOutput);

  m_rightPIDController.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
  m_rightPIDController.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
  m_rightPIDController.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
  m_rightPIDController.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);
  
  m_leftPIDController.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
  m_leftPIDController.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
  m_leftPIDController.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
  m_leftPIDController.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);

  // set inversions of drive so all motors spin together when given same inputs
  m_leftDriveLead.SetInverted(false);
  m_rightDriveLead.SetInverted(true);
}

void Drivetrain::ResetEncoders() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));
}

void Drivetrain::ResetIMU() {
  imu.Reset();
}

double Drivetrain::GetEncAvgDistance() {
  return ((m_leftEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor) + (m_rightEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor)) / 2.0;
}

units::meter_t Drivetrain::GetLeftDistance() {
  return m_leftEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
}

units::meter_t Drivetrain::GetRightDistance() {
  return m_rightEncoder.GetPosition() * RamseteConstants::kPositionConversionFactor * 1_m;
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
  return { units::meters_per_second_t(m_leftEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor),
            units::meters_per_second_t(m_rightEncoder.GetVelocity() * RamseteConstants::kVelocityConversionFactor) };
}

void Drivetrain::TankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts) {
  m_leftDriveLead.SetVoltage(leftVolts);
  m_rightDriveLead.SetVoltage(rightVolts);
}

void Drivetrain::RocketLeagueDrive(double straightInput, double reverseInput, double turnInput, bool limelightInput) {
  straightValue = reverseInput - straightInput;
  if (std::abs(straightValue) < DriveConstants::kDeadbandY) {
    straightValue = 0;
  }
  directionY = (straightValue >= 0) ? 1 : -1;
  straightTarget = DriveConstants::MAX_RPM * -std::pow(straightValue, 2) * directionY * DriveConstants::kDriveMultiplierY * boostMultiplier;
  
  turnValue = turnInput;

  directionX = (turnValue >= 0) ? 1 : -1;
  turnValue = std::pow(turnValue * DriveConstants::kDriveMultiplierX, 2) * directionX;
  turnTarget = DriveConstants::MAX_RPM * -turnValue;

  if (std::abs(turnValue) < DriveConstants::kDeadbandX) {
    if (std::abs(straightValue) < DriveConstants::kDeadbandY) {
      turnTarget = 0; //if turning, don't use drive straightening
    }
    else {
      turnTarget = (m_leftEncoder.GetVelocity() - m_rightEncoder.GetVelocity()) * DriveConstants::kDriveOffset;
      // Robot tends to curve to the left at 50RPM slower
    }
  }

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  float tx = table->GetNumber("tx", 0.0);
  float tv = table->GetNumber("tv" , 0.0);

  // Limelight button pressed
  if (limelightInput) {
    table->PutNumber("ledMode", LimelightConstants::LED_MODE_ON);
    table->PutNumber("camMode", LimelightConstants::TRACK_MODE_ON);
    limelight_state = LimelightConstants::STATE_ON;
    
    // Limelight has target, track and move robot
    if (tv == 1) {
      turnTarget = DriveConstants::kP * tx * DriveConstants::MAX_RPM;
    } 
    else {
      table->PutNumber("ledMode", LimelightConstants::LED_MODE_OFF);
      table->PutNumber("camMode", LimelightConstants::TRACK_MODE_OFF);
      limelight_state = LimelightConstants::STATE_OFF;
    }
  }

  // set lead PID controllers to desired velocities based on gamepad and limelight inputs
  m_leftPIDController.SetReference(straightTarget - turnTarget, rev::ControlType::kVelocity);
  m_rightPIDController.SetReference(straightTarget + turnTarget, rev::ControlType::kVelocity);
  
  frc::SmartDashboard::PutNumber("TurnVelocity", turnTarget);
  frc::SmartDashboard::PutNumber("ForwardVelocity", straightTarget);
  frc::SmartDashboard::PutNumber("LeftVelocityVariable", m_leftEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("RightVelocityVariable", m_rightEncoder.GetVelocity());
}

void Drivetrain::SetMultiplier(double multiplier) {
  boostMultiplier = multiplier;
}

void Drivetrain::Stop() {
  m_leftDriveLead.StopMotor();
  m_leftDriveFollowA.StopMotor();
  m_leftDriveFollowB.StopMotor();
  m_rightDriveLead.StopMotor();
  m_rightDriveFollowA.StopMotor();
  m_rightDriveFollowB.StopMotor();
}