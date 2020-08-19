#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <ctre/Phoenix.h>

#ifndef ARM_H
#define ARM_H

class Arm : public ValorSubsystem {
  public:
    Arm();
    Arm(frc::XboxController*);

    static Arm& GetInstance();

    void Periodic();

    void InitArm();

    void setDefaultState();
    void assessInputs();
    void assignOutputs();

    enum ArmState {
        DISABLED, 
        DISENGAGE, 
        MANUAL
    };

    struct x {
        ArmState arm_state;

        frc::Timer timer;
        bool disengage;
        double current_power;
        double curr_time;
        double step1_start_time;
        double step2_start_time;
    } state;

  private:
    TalonSRX armMtrLeft;
    TalonSRX armMtrRight;

    frc::XboxController* operator_controller;
};

#endif