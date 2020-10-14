/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SelectCommand.h>

#include <frc/XboxController.h>

#include "Drivetrain.h"
// #include "Arm.h"
#include "Shooter.h"
#include "Intake.h"
#include "Hopper.h"

#ifndef ROBOT_CONTAINER_H
#define ROBOT_CONTAINER_H

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
    public:
        RobotContainer();
  
        frc::XboxController m_GamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};
        frc::XboxController m_GamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        
        frc2::Command* GetAutonomousCommand();

        Drivetrain m_drivetrain;
        // Arm arm;
        Shooter m_shooter;
        Intake m_intake;
        Hopper m_hopper;

    private:

        // The enum used as keys for selecting the command to run.
        enum CommandSelector { ONE, TWO, THREE };

        // An example selector method for the selectcommand.  Returns the selector
        // that will select which command to run.  Can base this choice on logical
        // conditions evaluated at runtime.
        CommandSelector Select() { return ONE; }

        // The robot's subsystems and commands are defined here...

        // An example selectcommand.  Will select from the three commands based on the
        // value returned by the selector method at runtime.  Note that selectcommand
        // takes a generic type, so the selector does not have to be an enum; it could
        // be any desired type (string, integer, boolean, double...)
        frc2::SelectCommand<CommandSelector> m_exampleSelectCommand{
            [this] { return Select(); },
            // Maps selector values to commands
            std::pair{ONE, frc2::PrintCommand{"Command one was selected!"}},
            std::pair{TWO, frc2::PrintCommand{"Command two was selected!"}},
            std::pair{THREE, frc2::PrintCommand{"Command three was selected!"}}};

        void ConfigureButtonBindings();
};

#endif