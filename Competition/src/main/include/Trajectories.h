#include "ValorTrajectory.h"
#include <unordered_map>
#include "subsystems/Drivetrain.h"
#include <frc/geometry/Translation2d.h>

#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

class Trajectories {
    public:
        Trajectories();

        std::unordered_map<std::string, std::vector<ValorTrajectory>> GetAutosMap();

        std::unordered_map<std::string, std::vector<ValorTrajectory>> autos;

        std::vector<ValorTrajectory> eightBallPath;
        ValorTrajectory EBpath1;
        
        std::vector<ValorTrajectory> tenBallPath;
        ValorTrajectory TBpath1;
        ValorTrajectory TBpath2;
        ValorTrajectory TBpath3;
        ValorTrajectory TBpath4;
        ValorTrajectory TBpath5;
        ValorTrajectory TBpath6;

        // std::vector<ValorTrajectory> sixBallPath;
        // ValorTrajectory SBpath1;
        // ValorTrajectory SBpath2;

    private:
        
};

#endif