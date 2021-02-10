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
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Spindexer.h"

#ifndef VALOR_AUTO_H
#define VALOR_AUTO_H

class ValorAuto {
    public:
        ValorAuto(Drivetrain*, Intake*, Shooter*, Spindexer*);

        frc2::Command* getCurrentAuto();

        struct ValorTrajectory {
            frc::Trajectory trajectory;
            enum Action { Path, Shoot } action;
        };

    private:

        Drivetrain *drivetrain;
        Intake *intake;
        Shooter *shooter;
        Spindexer *spindexer;

        frc::DifferentialDriveKinematics kDriveKinematics;
        frc::SimpleMotorFeedforward<units::meters> kSimpleMotorFeedforward;
        frc::TrajectoryConfig kTrajectoryConfigForward;
        frc::TrajectoryConfig kTrajectoryConfigReverse;
        frc::DifferentialDriveVoltageConstraint kDifferentialDriveVoltageConstraint;

        std::unordered_map<std::string, std::vector<ValorTrajectory>> autos;
};

#endif