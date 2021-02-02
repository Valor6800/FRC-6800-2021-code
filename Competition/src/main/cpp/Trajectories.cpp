#include "Trajectories.h"

Trajectories::Trajectories() {
    /*
    SBpath1.action = ValorTrajectory::Path;
    SBpath2.action = ValorTrajectory::Path;

    SBpath1.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                                                                      {},
                                                                      frc::Pose2d(5_m, 0.2_m, frc::Rotation2d(-15_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigCenterLineF);

    SBpath2.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(5_m, 0.2_m, frc::Rotation2d(-15_deg)),
                                                                      {},
                                                                      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigReverse);
    sixBallPath.push_back(SBpath1);
    sixBallPath.push_back(SBpath2);

    autos.insert({"SixBallAuto", sixBallPath});
    */

    EBpath1.action = ValorTrajectory::Path;
    EBpath1.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m,0_m,frc::Rotation2d(0_deg)),
                                                                      { frc::Translation2d(1_m, -0.5_m) },
                                                                      frc::Pose2d(2_m, -0.5_m, frc::Rotation2d(0_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigForward);
    
    
    eightBallPath.push_back(EBpath1);

    autos.insert({"EightBallAuto", eightBallPath});

    TBpath1.action = ValorTrajectory::Path;
    TBpath2.action = ValorTrajectory::Path;
    TBpath3.action = ValorTrajectory::Path;
    TBpath4.action = ValorTrajectory::Path;
    TBpath5.action = ValorTrajectory::Path;
    TBpath6.action = ValorTrajectory::Path;

    /*
    const std::vector<frc::Pose2d> Auto3Traj1Poses = {frc::Pose2d(0_m,0_m,frc::Rotation2d(0_deg)),frc::Pose2d(3.7_m,(-7.64_m  + kStartPos),frc::Rotation2d(-60_deg))};
        //return
    const std::vector<frc::Pose2d> Auto3Traj2Poses = {Auto3Traj1Poses[1],frc::Pose2d(0_m,(-2.69_m + kStartPos),frc::Rotation2d(0_deg))};
    //Balls on the bar
    const std::vector<frc::Pose2d> Auto3Traj3Poses = {Auto3Traj2Poses[1],frc::Pose2d(3.3_m,(-3.04_m + kStartPos),frc::Rotation2d(-35_deg))};
    //ball pos: 2.9,0.5    3.3,0.7
        //return
    const std::vector<frc::Pose2d> Auto3Traj4Poses = {Auto3Traj3Poses[1],frc::Pose2d(1.5_m,(-kCenterline + kStartPos),frc::Rotation2d(0_deg))};
    //3 Trench balls         x's 23ft 5in(7.134m) 25ft 8in(7.823)        2.44m to wall tip of triangle
    const std::vector<frc::Translation2d> Auto3Traj5Translations = {frc::Translation2d(2.2_m,(-kCenterline + kStartPos))};  //center line needs to be adjusted
    const std::vector<frc::Pose2d> Auto3Traj5Poses = {frc::Pose2d(Auto3Traj4Poses[1]),frc::Pose2d(5_m,(-kCenterline + kStartPos),frc::Rotation2d(0_deg))};
    //ShootSpot
    const std::vector<frc::Pose2d> Auto3Traj6Poses = {Auto3Traj5Poses[1] ,frc::Pose2d(4_m,(-kCenterline + kStartPos -0.2_m),frc::Rotation2d(15_deg))};
    */

    TBpath1.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m,0_m,frc::Rotation2d(0_deg)),
                                                                      {},
                                                                      frc::Pose2d(3.7_m,(-7.64_m  + RamseteConstants::kStartPos),frc::Rotation2d(-60_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigForward);

    TBpath2.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(3.7_m,(-7.64_m  + RamseteConstants::kStartPos),frc::Rotation2d(-60_deg)),
                                                                      {},
                                                                      frc::Pose2d(0_m,(-2.69_m + RamseteConstants::kStartPos),frc::Rotation2d(0_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigReverse);

    TBpath3.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(0_m,(-2.69_m + RamseteConstants::kStartPos),frc::Rotation2d(0_deg)),
                                                                      {},
                                                                      frc::Pose2d(3.3_m,(-3.04_m + RamseteConstants::kStartPos),frc::Rotation2d(-35_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigForward);

    TBpath4.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(3.3_m,(-3.04_m + RamseteConstants::kStartPos),frc::Rotation2d(-35_deg)),
                                                                      {},
                                                                      frc::Pose2d(1.5_m,(-RamseteConstants::kCenterline + RamseteConstants::kStartPos),frc::Rotation2d(0_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigReverse);
    TBpath5.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(1.5_m,(-RamseteConstants::kCenterline + RamseteConstants::kStartPos),frc::Rotation2d(0_deg)),
                                                                      {frc::Translation2d(2.2_m,(-RamseteConstants::kCenterline + RamseteConstants::kStartPos))},
                                                                      frc::Pose2d(5_m,(-RamseteConstants::kCenterline + RamseteConstants::kStartPos),frc::Rotation2d(0_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigForward);
    TBpath6.trajectory = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(5_m,(-RamseteConstants::kCenterline + RamseteConstants::kStartPos),frc::Rotation2d(0_deg)),
                                                                      {},
                                                                      frc::Pose2d(4_m,(-RamseteConstants::kCenterline + RamseteConstants::kStartPos -0.2_m),frc::Rotation2d(15_deg)),
                                                                      Drivetrain::GetInstance().kTrajectoryConfigReverse);
                                                                       

    tenBallPath.push_back(TBpath1);
    tenBallPath.push_back(TBpath2);
    tenBallPath.push_back(TBpath3);
    tenBallPath.push_back(TBpath4);
    tenBallPath.push_back(TBpath5);
    tenBallPath.push_back(TBpath6);

    autos.insert({"TenBallAuto", tenBallPath});
}

std::unordered_map<std::string, std::vector<ValorTrajectory>> Trajectories::GetAutosMap() {
    return autos;
}