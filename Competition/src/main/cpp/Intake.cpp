#include "Intake.h"

Intake::Intake() : intakeMtr{IntakeConstants::VICTOR_ID_INTAKE}, 
             operatorController(NULL),
             driverController(NULL) {
    init();
}

void Intake::setController(frc::XboxController* controllerOperator, frc::XboxController* controllerDriver) {
    operatorController = controllerOperator;
    driverController = controllerDriver;
}

void Intake::init() {
    intakeMtr.SetInverted(false);
    
}

void Intake::setDefaultState() {
    state.intakeState = IntakeState::DISABLED;
    
    resetState();
}

void Intake::assessInputs() {
    // Prevent controller segfault
    if (!operatorController) {
        return;
    }
    if (!driverController) {
        return;
    }
    // if (std::abs(driverController->GetRawButton(9))) {
    //     state.intakeState = IntakeState::IN;
    // }
    // else if (std::abs(operatorController->GetRawButton(10))) {
    //     state.intakeState = IntakeState::OUT;
    // }
    if (operatorController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.intakeState = IntakeState::IN;
    }
    else if (driverController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.intakeState = IntakeState::OUT;
    }
    else {
        state.intakeState = IntakeState::DISABLED;
    }
    
}

void Intake::assignOutputs() {
    if (state.intakeState == IntakeState::IN) {
        state.currentPower = 1.0;
        

    }
    else if (state.intakeState == IntakeState::OUT) {
            state.currentPower = -1.0;
            

    }
    else  {
            state.currentPower = 0;  
          }    
      
    intakeMtr.Set(state.currentPower);
}
void Intake::resetState() {

}
