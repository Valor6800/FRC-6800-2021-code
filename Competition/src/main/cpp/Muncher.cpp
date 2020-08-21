#include "Muncher.h"

Muncher::Muncher() : munchMtr{MuncherConstants::VICTOR_ID_MUNCHER},
                    operatorController(NULL) {
    initMuncher();
}

void Muncher::setController(frc::XboxController* controller) {
    this->operatorController = controller;
}

void Muncher::initMuncher() {
    /* Might Be Needed Later */
}

void Muncher::setDefaultState() {
    state.m_state = MuncherState::DISABLED;
}

void Muncher::assessInputs() {
    if (!operatorController) {
        return;
    }

    state.buttonY = operatorController->GetYButton();

    if (state.buttonY) {
        state.m_state = MuncherState::MUNCH;
    }
}

void Muncher::assignOutputs() {
    if (state.m_state == MuncherState::DISABLED) {
        state.target = 0;
    }

    if (state.m_state == MuncherState::MUNCH) {
        if (state.buttonY) {
            state.target = 1;
        } else {
            state.target = 0;
        }
    }

    munchMtr.Set(state.target);
}