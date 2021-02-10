#include "Trajectories.h"

Trajectories::Trajectories(Drivetrain* train) :
        drivetrain(train),
        kDriveKinematics{DriveConstants::kTrackwidth},
        kSimpleMotorFeedforward{RamseteConstants::kS, RamseteConstants::kV, RamseteConstants::kA},
        kTrajectoryConfigForward{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
        kTrajectoryConfigReverse{RamseteConstants::kMaxSpeed, RamseteConstants::kMaxAcceleration},
        kDifferentialDriveVoltageConstraint{kSimpleMotorFeedforward, kDriveKinematics, 10_V} {

    kTrajectoryConfigForward.SetKinematics(kDriveKinematics);
    kTrajectoryConfigForward.AddConstraint(kDifferentialDriveVoltageConstraint);
    kTrajectoryConfigForward.SetReversed(false);

    kTrajectoryConfigReverse.SetKinematics(kDriveKinematics);
    kTrajectoryConfigReverse.AddConstraint(kDifferentialDriveVoltageConstraint);
    kTrajectoryConfigReverse.SetReversed(true);

    std::vector<ValorTrajectory> shoot3move5;

    ValorTrajectory shoot3;
    shoot3.action = ValorTrajectory::Shoot;

    ValorTrajectory pathMove5;
    pathMove5.action = ValorTrajectory::Path;
    pathMove5.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m,0_m,frc::Rotation2d(0_deg)),
                                                                      { frc::Translation2d(1_m, -0.5_m) },
                                                                      frc::Pose2d(2_m, -0.5_m, frc::Rotation2d(0_deg)),
                                                                      kTrajectoryConfigForward);
    
    
    shoot3move5.push_back(shoot3);
    shoot3move5.push_back(pathMove5);

    autos.insert({"Shoot3Move5", shoot3move5});
}

frc2::Command* Trajectories::getCurrentAuto() {

    // @TODO parse current auto from dashboard
    // Retreive the current auto from the autos map
    // From the current auto, iterate through each step of the auto
    // Construct a sequential command from each step.
    // If a step is a path, add a ramsete command
    // Else if the step is not a path, add a instant command to set the state

    // Below is an example of creating a ramsete command

    frc2::RamseteCommand ramseteCommand1(autos.at("Shoot3Move5")[1].trajectory,
                                    [&] () { return drivetrain->GetPose(); },
                                    frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                    kSimpleMotorFeedforward,
                                    kDriveKinematics,
                                    [&] { return drivetrain->GetWheelSpeeds(); },
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    [&] (auto left, auto right) { drivetrain->TankDriveVolts(left, right); },
                                    {drivetrain});
    return &ramseteCommand1;
}