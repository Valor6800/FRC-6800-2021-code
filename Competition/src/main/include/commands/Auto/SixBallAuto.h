// #pragma once

// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include <frc/kinematics/DifferentialDriveKinematics.h>
// #include <frc/controller/SimpleMotorFeedforward.h>
// #include "Trajectories.h"
// #include <frc2/command/RamseteCommand.h>
// #include <frc/controller/RamseteController.h>
// #include <frc/controller/PIDController.h>
// #include <frc2/command/RunCommand.h>
// #include <frc2/command/ParallelCommandGroup.h>
// #include <frc2/command/ParallelRaceGroup.h>
// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/WaitCommand.h>

// #include "subsystems/Drivetrain.h"
// #include "subsystems/Intake.h"
// #include "subsystems/Shooter.h"
// #include "subsystems/Hopper.h"

// #include "commands/ShootStart.h"

// #include "Constants.h"

// class SixBallAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, SixBallAuto> {
//  public:
//     SixBallAuto();

// private:
//     Drivetrain& m_drivetrain = Drivetrain::GetInstance();
//     Intake& m_intake = Intake::GetInstance();
//     Shooter& m_shooter = Shooter::GetInstance();
//     Hopper& m_hopper = Hopper::GetInstance();

//     Trajectories m_trajectories;

//     frc::Trajectory trajectory1;
//     frc::Trajectory trajectory2;
// };