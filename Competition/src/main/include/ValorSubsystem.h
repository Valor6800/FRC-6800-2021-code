/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

class ValorSubsystem : public frc2::Subsystem {
public:
  ValorSubsystem();

  void Periodic();

  static ValorSubsystem& GetInstance();

  // Rules:
  //   * Never read 'state', can only write 'state'
  virtual void setDefaultState();

  // Rules:
  //   * Never read 'state', can only write 'state'
  virtual void assessInputs();

  // Rules:
  //   * Never write another subsystem's 'state', only can read
  //   * Can read or write 'state'
  virtual void assignOutputs();
};
