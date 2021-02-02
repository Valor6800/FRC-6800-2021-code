#include <commands/Auto/HomeTrenchAuto.h>

HomeTrenchAuto::HomeTrenchAuto() {
    trajectory1 = m_trajectories.GetAutosMap().at("EightBallAuto")[0].trajectory;
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


    AddCommands(std::move(ramseteCommand1));
}