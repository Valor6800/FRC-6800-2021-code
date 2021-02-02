#include <commands/Auto/TenBallAuto.h>

TenBallAuto::TenBallAuto() {
    trajectory1 = m_trajectories.GetAutosMap().at("TenBallAuto")[0].trajectory;
    frc2::RamseteCommand ramseteCommand1(trajectory1,
                                        [&] () { return m_drivetrain.GetPose(); },
                                        frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                        m_drivetrain.kSimpleMotorFeedforward,
                                        m_drivetrain.kDriveKinematics,
                                        [&] { return m_drivetrain.GetWheelSpeeds(); },
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        [&] (auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
                                        {&Drivetrain::GetInstance()});

    trajectory2 = m_trajectories.GetAutosMap().at("TenBallAuto")[1].trajectory;
    frc2::RamseteCommand ramseteCommand2(trajectory2,
                                        [&] () { return m_drivetrain.GetPose(); },
                                        frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                        m_drivetrain.kSimpleMotorFeedforward,
                                        m_drivetrain.kDriveKinematics,
                                        [&] { return m_drivetrain.GetWheelSpeeds(); },
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        [&] (auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
                                        {&Drivetrain::GetInstance()});


    trajectory3 = m_trajectories.GetAutosMap().at("TenBallAuto")[2].trajectory;
    frc2::RamseteCommand ramseteCommand3(trajectory3,
                                        [&] () { return m_drivetrain.GetPose(); },
                                        frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                        m_drivetrain.kSimpleMotorFeedforward,
                                        m_drivetrain.kDriveKinematics,
                                        [&] { return m_drivetrain.GetWheelSpeeds(); },
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        [&] (auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
                                        {&Drivetrain::GetInstance()});


    trajectory4 = m_trajectories.GetAutosMap().at("TenBallAuto")[3].trajectory;
    frc2::RamseteCommand ramseteCommand4(trajectory4,
                                        [&] () { return m_drivetrain.GetPose(); },
                                        frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                        m_drivetrain.kSimpleMotorFeedforward,
                                        m_drivetrain.kDriveKinematics,
                                        [&] { return m_drivetrain.GetWheelSpeeds(); },
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        [&] (auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
                                        {&Drivetrain::GetInstance()});

    trajectory5 = m_trajectories.GetAutosMap().at("TenBallAuto")[4].trajectory;
    frc2::RamseteCommand ramseteCommand5(trajectory5,
                                        [&] () { return m_drivetrain.GetPose(); },
                                        frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                        m_drivetrain.kSimpleMotorFeedforward,
                                        m_drivetrain.kDriveKinematics,
                                        [&] { return m_drivetrain.GetWheelSpeeds(); },
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        [&] (auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
                                        {&Drivetrain::GetInstance()});

    trajectory6 = m_trajectories.GetAutosMap().at("TenBallAuto")[5].trajectory;
    frc2::RamseteCommand ramseteCommand6(trajectory6,
                                        [&] () { return m_drivetrain.GetPose(); },
                                        frc::RamseteController(RamseteConstants::kRamseteB, RamseteConstants::kRamseteZeta),
                                        m_drivetrain.kSimpleMotorFeedforward,
                                        m_drivetrain.kDriveKinematics,
                                        [&] { return m_drivetrain.GetWheelSpeeds(); },
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        frc2::PIDController(RamseteConstants::kPDriveVel, 0, 0),
                                        [&] (auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
                                        {&Drivetrain::GetInstance()});

    AddCommands(std::move(ramseteCommand1), std::move(ramseteCommand2), std::move(ramseteCommand3), std::move(ramseteCommand4), std::move(ramseteCommand5), std::move(ramseteCommand6));

    // AddCommands(frc2::ParallelCommandGroup(
    //     frc2::SequentialCommandGroup(std::move(ramseteCommand1), std::move(ramseteCommand2), std::move(ramseteCommand3), std::move(ramseteCommand4), std::move(ramseteCommand5), std::move(ramseteCommand6)),
    //     frc2::RunCommand([&] { m_intake.SetIntakePower(0.8); }) ));
}