/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drivetrain.h"

Drivetrain::Drivetrain() {}

Drivetrain& Drivetrain::GetInstance()
{
  // Guaranteed to be destroyed. Instantiated on first use.
  static Drivetrain instance;
  return instance;
}

void Drivetrain::Periodic() {
  // Update odometry
}