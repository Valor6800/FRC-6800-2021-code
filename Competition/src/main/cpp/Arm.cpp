// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #include "Arm.h"

// Arm::Arm() : ValorSubsystem(),
//              armMtrLeft{ArmConstants::TALON_ID_LEFT_ARM}, 
//              armMtrRight{ArmConstants::TALON_ID_RIGHT_ARM},
//              operatorController(NULL) {
// }

// void Arm::setController(frc::XboxController* controller) {
//     operatorController = controller;
// }

// void Arm::init() {
//     armMtrLeft.ConfigFactoryDefault();
//     armMtrRight.ConfigFactoryDefault();

//     armMtrLeft.SetNeutralMode(NeutralMode::Brake);
//     armMtrRight.SetNeutralMode(NeutralMode::Brake);

//     armMtrLeft.SetInverted(false);
//     armMtrRight.SetInverted(false);
// }

// void Arm::setDefaultState() {
//     state.armState = ArmState::DISABLED;

//     resetState();
// }

// void Arm::assessInputs() {
//     // Prevent controller segfault
//     if (!operatorController) {
//         return;
//     }

//     if (std::abs(operatorController->GetY(frc::GenericHID::kLeftHand)) > 0.05) {
//         state.armState = ArmState::MANUAL;
        
//         state.leftJoystickY = operatorController->GetY(frc::GenericHID::kLeftHand);
//     }
// }

// void Arm::assignOutputs() {
//     if (state.disengage) {
//         state.armState = ArmState::DISENGAGE;
//     }

//     if (state.armState == ArmState::DISENGAGE) {
//         state.currentTime = state.timer.GetFPGATimestamp();

//         if (state.step1_startTime == -1) {
//             state.step1_startTime = state.currentTime;
//         }
//         else if (state.currentTime - state.step1_startTime <= 0.2) {
//             state.currentPower = 0.4;
//         }
//         else if (state.step2_startTime == -1) {
//             state.step2_startTime = state.currentTime;
//         }
//         else if (state.currentTime - state.step2_startTime <= 0.3) {
//             state.currentPower = 0.042;
//         }
//         else {
//             state.currentPower = 0;
//             resetState();
//         }
//     }
//     else if (state.armState == ArmState::MANUAL) {
//         if (std::abs(state.leftJoystickY) <= 0.05) {
//             state.currentPower = 0;
//         }
//         else if (state.leftJoystickY < -0.05) {
//             state.currentPower = -0.5;
//         }
//         else if (state.leftJoystickY > 0.05 && state.leftJoystickY < 0.85) {
//             state.currentPower = -0.042;
//         }
//         else {
//             state.currentPower = 0.1;
//         }
//     }
//     else {
//         state.currentPower = 0;
//     }
//     armMtrLeft.Set(ControlMode::PercentOutput, state.currentPower);
//     armMtrRight.Set(ControlMode::PercentOutput, state.currentPower);
// }

// void Arm::setDisengage(bool disengage) {
//     state.disengage = disengage;
// }

// void Arm::resetState() {
//     setDisengage(false);
//     state.step1_startTime = -1;
//     state.step2_startTime = -1;
// }