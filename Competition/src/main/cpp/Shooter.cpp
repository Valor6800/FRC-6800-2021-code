/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Shooter.h"

Shooter::Shooter(frc::XboxController* controller) :
                 operator_controller(controller)
{}

void Shooter::setDefaultState()
{
    state.shooter_state = ShooterState::DISABLED;
    state.hood_state = HoodState::HOOD_DOWN;
}

void Shooter::assessInputs()
{

    // Operator holding the shoot button
    if (operator_controller->GetStartButton()) {
        state.shooter_state = ShooterState::SHOOTING;
        state.hood_state = HoodState::HOOD_UP;
    }

    // Operator holding the stop button (overrides shoot)
    if (operator_controller->GetBackButton()) {
        state.shooter_state = ShooterState::DISABLED;
        state.hood_state = HoodState::HOOD_DOWN;
    }
}

void Shooter::assignOutputs()
{
    // Save previous speed for this loop, set previous state to current
    state.previous_speed = state.current_speed;

    // Decision tree for shooter state

    // State: Shooting
    if (state.shooter_state == ShooterState::SHOOTING) {
        state.current_speed = 0.75;

        // Potentially use this for PD control?
        double verror = state.current_speed - state.previous_speed;

    // State: Disabled
    } else {
        state.current_speed = 0;
    }
    
    // motor.setOutput(state.current_speed);
}
