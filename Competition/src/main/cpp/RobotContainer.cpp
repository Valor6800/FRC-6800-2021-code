/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

RobotContainer::RobotContainer() :
        m_auto(&m_drivetrain, &m_intake, &m_shooter, &m_spindexer) {
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
     m_drivetrain.setController(&m_GamepadDriver);
     m_spindexer.setController(&m_GamepadOperator);
     m_lift.setController(&m_GamepadOperator);
     m_intake.setControllers(&m_GamepadOperator, &m_GamepadDriver);
     m_shooter.setController(&m_GamepadOperator);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_auto.getCurrentAuto();
}