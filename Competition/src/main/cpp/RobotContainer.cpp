/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    m_drivetrain.setController(&m_GamepadDriver);
    // m_arm.setController(&m_GamepadOperator);
    // m_shooter.setController(&m_GamepadOperator);
    // m_intake.setController(&m_GamepadOperator, &m_GamepadDriver);
    // m_hopper.setController(&m_GamepadOperator, &m_GamepadDriver);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    // Run the select command in autonomous
    return &m_exampleSelectCommand;
}
