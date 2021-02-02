#include <iostream> 
#include <unordered_map>
#include <vector>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"

#ifndef VALOR_TRAJECTORY_H
#define VALOR_TRAJECTORY_H

struct ValorTrajectory {
    frc::Trajectory trajectory;
    enum Action { Path, Shoot } action;
};

#endif