#include "Hopper.h"

Hopper::Hopper() : hopperMtr{HopperConstants::VICTOR_ID_HOPPER}, 
            throatMtr{HopperConstants::VICTOR_ID_THROAT},
             operatorController(NULL),
             driverController(NULL) {
    initHopper();
}

void Hopper::setController(frc::XboxController* controllerOperator, frc::XboxController* controllerDriver) {
    operatorController = controllerOperator;
    driverController = controllerDriver;
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
    if (!driverController) {
        return;
    }
    // if (std::abs(driverController->GetRawButton(9))) {
    //     state.hopperState = HopperState::FORWARD;
    // }
    // else if (std::abs(operatorController->GetRawButton(10))) {
    //     state.hopperState = HopperState::REVERSE;
    // }
    if (driverController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.hopperState = HopperState::FORWARD;
    }
    else if (operatorController->GetBumper(frc::GenericHID::kRightHand)) {
        state.hopperState = HopperState::REVERSE;
    }
    else {
        state.hopperState = HopperState::DISABLED;
    }
    
}

void Hopper::assignOutputs() {
    if (state.hopperState == HopperState::FORWARD) {
        state.currentHopperPower = 1.0;
        state.currentThroatPower = 1.0;

    }
    else if (state.hopperState == HopperState::REVERSE) {
            state.currentHopperPower = -1.0;
            state.currentThroatPower = -1.0;

    }
    else  {
            state.currentHopperPower = 0;
            state.currentThroatPower = 0;
    }    
      
    hopperMtr.Set(state.currentHopperPower);
    throatMtr.Set(state.currentThroatPower);
}
void Hopper::resetState() {

}