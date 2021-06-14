#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain* _drivetrain, Intake* _intake, Shooter* _shooter, Spindexer* _spindexer) :
        drivetrain(_drivetrain),
        intake(_intake),
        shooter(_shooter),
        spindexer(_spindexer),
        kDriveKinematics{DriveConstants::kTrackwidth},
        kSimpleMotorFeedforward{RamseteConstants::kS, RamseteConstants::kV, RamseteConstants::kA},
        kTrajectoryConfigForward{RamseteConstants::kMaxForwardSpeed, RamseteConstants::kMaxAcceleration},
        kTrajectoryConfigReverse{RamseteConstants::kMaxReverseSpeed, RamseteConstants::kMaxAcceleration},
        kDifferentialDriveVoltageConstraint{kSimpleMotorFeedforward, kDriveKinematics, 10_V} {

    table = nt::NetworkTableInstance::GetDefault().GetTable("Auto");
    table->PutNumber("Current Auto", 0);

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
    frc2::InstantCommand cmd_track = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TRACK;
    });
    frc2::InstantCommand cmd_stopShoot = frc2::InstantCommand( [&] {
        spindexer->state.drumState = Spindexer::DrumState::LOW;
    });

    //SHOOT 3 MOVE 5 AUTO
    auto traj_move5 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m, 0_m,frc::Rotation2d(0_deg)),
                                                                    { frc::Translation2d(2_m, 1.6_m)},
                                                                    frc::Pose2d(5.6_m, 2_m, frc::Rotation2d(0_deg)),
                                                                    kTrajectoryConfigForward);

    auto traj_reverse_move5 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(5.6_m, 2_m, frc::Rotation2d(0_deg)),
                                                                    { },
                                                                    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                                                                    kTrajectoryConfigReverse);

    // SHOOT 3 MOVE 5 OFFSET POSITION
    auto traj_move5_offset = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m, 0_m,frc::Rotation2d(0_deg)),
                                                                    { frc::Translation2d(2_m, 0.6_m)},
                                                                    frc::Pose2d(5.6_m, 1_m, frc::Rotation2d(0_deg)),
                                                                    kTrajectoryConfigForward);

    auto traj_reverse_move5_offset = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(5.6_m, 1_m, frc::Rotation2d(0_deg)),
                                                                    { },
                                                                    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                                                                    kTrajectoryConfigReverse);

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

    frc2::RamseteCommand cmd_reverse_move5(traj_reverse_move5,
                                    [&] () { return drivetrain->GetPose(); },
                                    frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                    kSimpleMotorFeedforward,
                                    kDriveKinematics,
                                    [&] { return drivetrain->GetWheelSpeeds(); },
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    [&] (auto left, auto right) { drivetrain->TankDriveVolts(left, right); },
                                    {drivetrain});

    frc2::RamseteCommand cmd_move5_offset(traj_move5_offset,
                                    [&] () { return drivetrain->GetPose(); },
                                    frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                    kSimpleMotorFeedforward,
                                    kDriveKinematics,
                                    [&] { return drivetrain->GetWheelSpeeds(); },
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                    [&] (auto left, auto right) { drivetrain->TankDriveVolts(left, right); },
                                    {drivetrain});

    frc2::RamseteCommand cmd_reverse_move5_offset(traj_reverse_move5_offset,
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
                             frc2::WaitCommand((units::second_t)1.5),
                             cmd_intake,
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)1.5),
                             cmd_stopShoot,
                             std::move(cmd_move5),
                             std::move(cmd_reverse_move5),
                             cmd_track,
                            frc2::WaitCommand((units::second_t)1),
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)2),
                             cmd_stopShoot,
                             cmd_spoolDown);
    autos["Shoot3Move5"] = shoot3move5;

    frc2::SequentialCommandGroup *shoot3move5Offset = new frc2::SequentialCommandGroup();
    shoot3move5Offset->AddCommands(cmd_spoolUp,
                             frc2::WaitCommand((units::second_t)1.5),
                             cmd_intake,
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)1.5),
                             cmd_stopShoot,
                             std::move(cmd_move5_offset),
                             std::move(cmd_reverse_move5_offset),
                             cmd_track,
                             frc2::WaitCommand((units::second_t)1),
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)2),
                             cmd_stopShoot,
                             cmd_spoolDown);
    autos["Shoot3Move5Offset"] = shoot3move5Offset;

    frc2::SequentialCommandGroup *shoot3 = new frc2::SequentialCommandGroup();
    shoot3->AddCommands(cmd_spoolUp,
                        frc2::WaitCommand((units::second_t)2),
                        cmd_shoot,
                        frc2::WaitCommand((units::second_t)3),
                        cmd_stopShoot,
                        cmd_spoolDown);
    autos["Shoot3"] = shoot3;

    frc2::SequentialCommandGroup *shoot3move5move5 = new frc2::SequentialCommandGroup();
    shoot3move5move5->AddCommands(cmd_spoolUp,
                             frc2::WaitCommand((units::second_t)1.5),
                             cmd_intake,
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)1.5),
                             cmd_stopShoot,
                             std::move(cmd_move5),
                             std::move(cmd_reverse_move5),
                             cmd_track,
                            frc2::WaitCommand((units::second_t)1),
                             cmd_shoot,
                             frc2::WaitCommand((units::second_t)2),
                             cmd_stopShoot,
                             cmd_spoolDown,
                             std::move(cmd_move5));
    autos["Shoot3Move5Move5"] = shoot3move5move5;
}

frc2::Command* ValorAuto::getCurrentAuto() {

    int selectedAuto = table->GetNumber("Current Auto", 0);
    std::string auto_name = "Shoot3Move5";
    if (selectedAuto == 1) {
        auto_name = "Shoot3Move5Offset";
    }
    else if (selectedAuto == 2) {
        auto_name = "Shoot3";
    }
    else if (selectedAuto == 3) {
        auto_name = "Shoot3Move5Move5";
    }
    auto selected_trajectories = autos[auto_name];
    return selected_trajectories;
}