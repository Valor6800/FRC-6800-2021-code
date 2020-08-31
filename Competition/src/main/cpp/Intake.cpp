#include "Intake.h"

Intake::Intake() : ValorSubsystem(),
             intakeMtr{IntakeConstants::VICTOR_ID_INTAKE}, 
             operatorController(NULL),
             driverController(NULL) {
    
}

void Intake::setController(frc::XboxController* controllerOperator, frc::XboxController* controllerDriver) {
    operatorController = controllerOperator;
    driverController = controllerDriver;
}

void Intake::init() {
    intakeMtr.SetInverted(false);
    
}

void Intake::setDefaultState() {    
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
    state.intakeState = IntakeState::DISABLED;
}
