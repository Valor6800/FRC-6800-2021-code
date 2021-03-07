/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Spindexer.h"
#include "subsystems/Lift.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

#include "ValorAuto.h"
#include "Constants.h"

#ifndef ROBOT_CONTAINER_H
#define ROBOT_CONTAINER_H

class RobotContainer {
    public:
        RobotContainer();
        frc2::Command* GetAutonomousCommand();

        frc::XboxController m_GamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};
        frc::XboxController m_GamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};

        Drivetrain m_drivetrain;
        Spindexer m_spindexer;
        Lift m_lift;
        Intake m_intake;
        Shooter m_shooter;

        frc::SendableChooser<wpi::StringRef> m_chooser;

    private:
        ValorAuto m_auto;
        void ConfigureButtonBindings();
};

#endif
