#include "Hopper.h"

Hopper::Hopper() : ValorSubsystem(),
             hopperMtr{HopperConstants::VICTOR_ID_HOPPER}, 
             throatMtr{HopperConstants::VICTOR_ID_THROAT},
             operatorController(NULL),
             driverController(NULL) {
}

void Hopper::setController(frc::XboxController* controllerOperator, frc::XboxController* controllerDriver) {
    operatorController = controllerOperator;
    driverController = controllerDriver;
}

void Hopper::init() {
    hopperMtr.SetInverted(false);
    
}

void Hopper::setDefaultState() {
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
    else if (operatorController->GetBumper(frc::GenericHID::kRightHand)) {
        state.hopperState = HopperState::REVERSE;
    }
    if (driverController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.hopperState = HopperState::FORWARD;
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
    state.hopperState = HopperState::DISABLED;
}