#include "subsystems/Lift.h"

Lift::Lift() : ValorSubsystem(),
                        leftMotor{LiftConstants::LEFT_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        rightMotor{LiftConstants::RIGHT_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        limitSwitch{LiftConstants::LIMIT_DIO},
                        pot{LiftConstants::POT_ANOLOG_PORT, LiftConstants::POT_RANGE_SCALE, LiftConstants::POT_RANGE_OFFSET} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    liftTable = nt::NetworkTableInstance::GetDefault().GetTable("Lift");
    liftTable->GetEntry("Lift Speed Out").SetDouble(0.0);
    liftTable->GetEntry("Lift Speed In").SetDouble(0.0);
}

void Lift::init() {
    leftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftMotor.SetInverted(false);
    rightMotor.SetInverted(false);
    
}

void Lift::setController(frc::XboxController* controller) {
    operatorController = controller;
}

void Lift::setDefaultState() {
    state.liftState = LiftState::DISABLED;
    resetState();
}

void Lift::resetState() {

}

void Lift::assessInputs() {
    state.limit = limitSwitch.Get();

    if (!operatorController) {
        return;
    }

    if (operatorController->GetTriggerAxis(frc::GenericHID::kRightHand) > 0.2) {
        state.liftState = LiftState::RETRACT;
        
    } else {
        if (operatorController->GetYButton()) {
            state.liftState = LiftState::EXTEND;
            
        } else {
            state.liftState = LiftState::DISABLED;
        }
    }
}

void Lift::assignOutputs() {
    state.liftState == LiftState::RETRACT ? frc::SmartDashboard::PutString("State", "Retract") : state.liftState == LiftState::EXTEND ? frc::SmartDashboard::PutString("State", "Extend") : frc::SmartDashboard::PutString("State", "Disabled");
    state.powerIn = liftTable->GetEntry("Lift Speed In").GetDouble(0.0);
    state.powerOut = liftTable->GetEntry("Lift Speed Out").GetDouble(0.0);

    if (state.liftState == LiftState::DISABLED) {
        leftMotor.Set(0);
        rightMotor.Set(0);
    } else {
        if (state.limit || state.liftState != LiftState::RETRACT) {
            if (state.liftState == LiftState::EXTEND) {
                //Motion Profiling goes here :)
                state.target = state.powerOut; //set motion profiling target to the output of the motion profiling. (Temp until Motion profiling)

                leftMotor.Set(state.target);
                rightMotor.Set(state.target);
            } else {
                leftMotor.Set(0);
                rightMotor.Set(0);
            }
        } else {
                leftMotor.Set(state.powerIn);
                rightMotor.Set(state.powerIn);
        } 
    }
}