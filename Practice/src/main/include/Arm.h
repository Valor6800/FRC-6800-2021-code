// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include "ValorSubsystem.h"
// #include "Constants.h"

// #include <frc/XboxController.h>
// #include <frc/Timer.h>
// #include <ctre/Phoenix.h>

// #ifndef ARM_H
// #define ARM_H

// class Arm : public ValorSubsystem {
//     public:
//         Arm();
//         void setController(frc::XboxController*);

//         void init();

//         void setDefaultState();
//         void assessInputs();
//         void assignOutputs();

//         void setDisengage(bool disengage);
        
//         void resetState();

//         enum ArmState {
//             DISABLED, 
//             DISENGAGE, 
//             MANUAL
//         };

//         struct x {
//             ArmState armState;

//             frc::Timer timer;
//             bool disengage;
//             double currentPower;
//             double currentTime;
//             double step1_startTime;
//             double step2_startTime;

//             double leftJoystickY;
//         } state;

//     private:
//         TalonSRX armMtrLeft;
//         TalonSRX armMtrRight;

//         frc::XboxController* operatorController;
// };

// #endif