#include "Hopper.h"

Hopper::Hopper() : hopperMtr{HopperConstants::VICTOR_ID_HOPPER}, 
            throatMtr{HopperConstants::VICTOR_ID_THROAT},
             operatorController(NULL) {
    initHopper();
}

void Hopper::setController(frc::XboxController* controller) {
    operatorController = controller;
    
}

void Hopper::initHopper() {
    hopperMtr.SetInverted(false);
    
}

void Hopper::setDefaultState() {
    state.hopperState = HopperState::DISABLED;

    resetState();
}

void Hopper::assessInputs() {
    // Prevent controller segfault
    if (!operatorController) {
        return;
    }

    if (std::abs(driverController->GetRawButton(9))) {
        state.hopperState = HopperState::FORWARD;
    }
    else if (std::abs(operatorController->GetRawButton(10))) {
        state.hopperState = HopperState::REVERSE;
    }
    
}

void Hopper::assignOutputs() {
    if (state.hopperState == HopperState::FORWARD) {
        state.currentPower = 1.0;
    }
    else if (state.hopperState == HopperState::REVERSE) {
            state.currentPower = -1.0;
    }
    else if (state.hopperState == HopperState::DISABLED) {
            state.currentPower = 0;
        
    hopperMtr.Set(state.currentPower);
    throatMtr.Set(state.currentPower);
}
}
