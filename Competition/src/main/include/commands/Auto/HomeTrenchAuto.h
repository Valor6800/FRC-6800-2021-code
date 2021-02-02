#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include "Trajectories.h"
#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>

#include "subsystems/Drivetrain.h"

#include "Constants.h"

class HomeTrenchAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, HomeTrenchAuto> {
 public:
    HomeTrenchAuto();

private:
    Drivetrain& m_drivetrain = Drivetrain::GetInstance();

    Trajectories m_trajectories;

    frc::Trajectory trajectory1;
};