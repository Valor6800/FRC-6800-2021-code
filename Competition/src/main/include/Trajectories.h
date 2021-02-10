#include "ValorTrajectory.h"
#include <unordered_map>
#include "subsystems/Drivetrain.h"
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>

#include "subsystems/Drivetrain.h"

#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

class Trajectories {
    public:
        Trajectories(Drivetrain*);

        frc2::Command* getCurrentAuto();

    private:

        Drivetrain *drivetrain;

        frc::DifferentialDriveKinematics kDriveKinematics;
        frc::SimpleMotorFeedforward<units::meters> kSimpleMotorFeedforward;
        frc::TrajectoryConfig kTrajectoryConfigForward;
        frc::TrajectoryConfig kTrajectoryConfigReverse;
        frc::DifferentialDriveVoltageConstraint kDifferentialDriveVoltageConstraint;

        std::unordered_map<std::string, std::vector<ValorTrajectory>> autos;
};

#endif