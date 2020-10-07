// #include "Drivetrain2.h"

// Drivetrain2::Drivetrain2() : ValorSubsystem(),
//                                    leftDriveLead{DriveConstants::CAN_ID_LEFT_LEAD, rev::CANSparkMax::MotorType::kBrushless},
//                                    leftDriveFollowA{DriveConstants::CAN_ID_LEFT_FOLLOW_A, rev::CANSparkMax::MotorType::kBrushless},
//                            leftDriveFollowB{DriveConstants::CAN_ID_LEFT_FOLLOW_B, rev::CANSparkMax::MotorType::kBrushless},
//                            rightDriveLead{DriveConstants::CAN_ID_RIGHT_LEAD, rev::CANSparkMax::MotorType::kBrushless},
//                            rightDriveFollowA{DriveConstants::CAN_ID_RIGHT_FOLLOW_A, rev::CANSparkMax::MotorType::kBrushless},
//                            rightDriveFollowB{DriveConstants::CAN_ID_RIGHT_FOLLOW_B, rev::CANSparkMax::MotorType::kBrushless}, 
//                         //    leftEnc{leftDriveLead},
//                         //    rightEnc{rightDriveLead},
//                         //    leftPID{leftDriveLead},
//                         //    rightPID{rightDriveLead},
//                            driverController(NULL) {
//     frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);


// }

// void Drivetrain2::init() {
//     leftDriveLead.RestoreFactoryDefaults();
//     leftDriveFollowA.RestoreFactoryDefaults();
//     leftDriveFollowB.RestoreFactoryDefaults();
//     rightDriveLead.RestoreFactoryDefaults();
//     rightDriveFollowA.RestoreFactoryDefaults();
//     rightDriveFollowB.RestoreFactoryDefaults();

//     leftDriveLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
//     leftDriveFollowA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
//     leftDriveFollowB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
//     rightDriveLead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
//     rightDriveFollowA.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
//     rightDriveFollowB.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

//     leftDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);
//     rightDriveLead.Follow(rev::CANSparkMax::kFollowerDisabled, false);

//     leftDriveFollowA.Follow(leftDriveLead);
//     leftDriveFollowB.Follow(leftDriveLead);

//     rightDriveFollowA.Follow(rightDriveLead);
//     rightDriveFollowB.Follow(rightDriveLead);

//     // leftPID.SetP(DriveConstants::kP);
//     // leftPID.SetI(DriveConstants::kI);
//     // leftPID.SetD(DriveConstants::kD);
//     // leftPID.SetIZone(DriveConstants::kIz);
//     // leftPID.SetFF(DriveConstants::kFF);
//     // leftPID.SetOutputRange(DriveConstants::kMinOutput, DriveConstants::kMaxOutput);

//     // rightPID.SetP(DriveConstants::kP);
//     // rightPID.SetI(DriveConstants::kI);
//     // rightPID.SetD(DriveConstants::kD);
//     // rightPID.SetIZone(DriveConstants::kIz);
//     // rightPID.SetFF(DriveConstants::kFF);
//     // rightPID.SetOutputRange(DriveConstants::kMinOutput, DriveConstants::kMaxOutput);

//     // rightPID.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
//     // rightPID.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
//     // rightPID.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
//     // rightPID.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);
    
//     // leftPID.SetSmartMotionMaxVelocity(DriveConstants::kMaxVel);
//     // leftPID.SetSmartMotionMinOutputVelocity(DriveConstants::kMinVel);
//     // leftPID.SetSmartMotionMaxAccel(DriveConstants::kMaxAccel);
//     // leftPID.SetSmartMotionAllowedClosedLoopError(DriveConstants::kAllError);

//     leftDriveLead.SetInverted(false);
//     rightDriveLead.SetInverted(true);
// }

// void Drivetrain2::setController(frc::XboxController* controller) {
//     driverController = controller;
// }

// void Drivetrain2::setDefaultState() {

// }

// void Drivetrain2::assessInputs() {

// }

// void Drivetrain2::assignOutputs() {
//     if (std::abs(driverController->GetTriggerAxis(frc::GenericHID::kRightHand)) > 0.05) {
//         leftDriveLead.Set(driverController->GetTriggerAxis(frc::GenericHID::kRightHand));
//         rightDriveLead.Set(-driverController->GetTriggerAxis(frc::GenericHID::kRightHand));
//     }
//     else {
//         leftDriveLead.Set(0);
//         leftDriveFollowA.Set(0);
//         leftDriveFollowB.Set(0);
//         rightDriveLead.Set(0);
//         rightDriveFollowA.Set(0);
//         rightDriveFollowB.Set(0);
//     }
// }

// void Drivetrain2::resetState() {

// }