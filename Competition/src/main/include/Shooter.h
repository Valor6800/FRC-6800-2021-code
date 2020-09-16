// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include "ValorSubsystem.h"

// #include <frc/XboxController.h>

// class Shooter : public ValorSubsystem {
// public:
//   Shooter(frc::XboxController*);

//   void setDefaultState();
//   void assessInputs();
//   void assignOutputs();

//   enum ShooterState {
//     DISABLED,
//     SHOOTING
//   };

//   enum HoodState {
//     HOOD_UP,
//     HOOD_DOWN
//   };

//   struct x {
//     double current_speed;
//     double previous_speed;

//     ShooterState shooter_state;
//     HoodState hood_state;
//   } state;

// private:
//   frc::XboxController* operator_controller;
// };
