#include "subsystems/Lift.h"

Lift::Lift() : ValorSubsystem(),
                        leftMotor{LiftConstants::LEFT_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        rightMotor{LiftConstants::RIGHT_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        limitSwitch{LiftConstants::LIMIT_DIO} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    liftTable = nt::NetworkTableInstance::GetDefault().GetTable("Lift");
    liftTable->GetEntry("Lift Speed").SetDouble(0.0);
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

    if (operatorController->GetBumper(frc::GenericHID::kRightHand)) {
        state.liftState = LiftState::RETRACT;
        state.power = -1 * liftTable->GetEntry("Lift Speed").GetDouble(0.0);
    } else {
        if (operatorController->GetYButton()) {
            state.liftState = LiftState::EXTEND;
            state.power = liftTable->GetEntry("Lift Speed").GetDouble(0.0);
            //Motion Profiling goes here :)
            state.target = state.power; //set motion profiling target to the output of the motion profiling.
        } else {
            state.liftState = LiftState::DISABLED;
        }
    }
}

void Lift::assignOutputs() {
    state.liftState == LiftState::RETRACT ? frc::SmartDashboard::PutString("State", "Retract") : state.liftState == LiftState::EXTEND ? frc::SmartDashboard::PutString("State", "Extend") : frc::SmartDashboard::PutString("State", "Disabled");

    if (state.liftState == LiftState::DISABLED) {
        leftMotor.Set(0);
        rightMotor.Set(0);
    } else {
        if (state.limit || state.liftState != LiftState::RETRACT) {
            if (state.liftState == LiftState::EXTEND) {
                leftMotor.Set(state.target);
                rightMotor.Set(state.target);
            } else {
                leftMotor.Set(0);
                rightMotor.Set(0);
            }
        } else {
            if (state.liftState == LiftState::RETRACT) {
                leftMotor.Set(state.power);
                rightMotor.Set(state.power);
            }
        } 
    }
}