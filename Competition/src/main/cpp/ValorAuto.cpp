#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain* _drivetrain, Intake* _intake, Shooter* _shooter, Spindexer* _spindexer) :
        drivetrain(_drivetrain),
        intake(_intake),
        shooter(_shooter),
        spindexer(_spindexer),
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

    frc2::InstantCommand cmd_spoolUp = frc2::InstantCommand( [&] { shooter->state.shooterState = true; } );
    frc2::InstantCommand cmd_spoolDown = frc2::InstantCommand( [&] { shooter->state.shooterState = false; } );
    frc2::InstantCommand cmd_intake = frc2::InstantCommand( [&] {
        intake->state.deployState = true;
        intake->state.intakeState = Intake::IntakeState::FORWARD;
    });
    frc2::InstantCommand cmd_shoot = frc2::InstantCommand( [&] {
        spindexer->state.drumState = Spindexer::DrumState::HIGH;
    });
    frc2::InstantCommand cmd_stopShoot = frc2::InstantCommand( [&] {
        spindexer->state.drumState = Spindexer::DrumState::LOW;
    });

    auto traj_move5 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m, 0_m,frc::Rotation2d(0_deg)),
                                                                      {},
                                                                      frc::Pose2d(5_m, 0.1_m, frc::Rotation2d(0_deg)),
                                                                      kTrajectoryConfigForward);

    frc2::RamseteCommand cmd_move5(traj_move5,
                                    [&] () { return drivetrain->GetPose(); },
                                    frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                    kSimpleMotorFeedforward,
                                    kDriveKinematics,
                                    [&] { return drivetrain->GetWheelSpeeds(); },
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    [&] (auto left, auto right) { drivetrain->TankDriveVolts(left, right); },
                                    {drivetrain});

    frc2::SequentialCommandGroup *shoot3move5 = new frc2::SequentialCommandGroup();
    shoot3move5->AddCommands(cmd_spoolUp,
                             frc2::WaitCommand((units::second_t)2),
                             cmd_intake,
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)2),
                             cmd_spoolDown,
                             cmd_stopShoot,
                             std::move(cmd_move5));
    autos["Shoot3Move5"] = shoot3move5;

}

frc2::Command* ValorAuto::getCurrentAuto() {

    std::string auto_name = "Shoot3Move5"; // @TODO retrieve from dashboard
    auto selected_trajectories = autos[auto_name];
    return selected_trajectories;
}