#include "subsystems/Intake.h"

Intake::Intake() : ValorSubsystem(),
                        motor{IntakeConstants::MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        compressor{IntakeConstants::COMPRESSOR_PCM_ID},
                        leftSolenoid{IntakeConstants::LEFT_SOLENOID_FORWARD_PCM_CAN_ID,IntakeConstants::LEFT_SOLENOID_REVERSE_PCM_CAN_ID},
                        rightSolenoid{IntakeConstants::RIGHT_SOLENOID_FORWARD_PCM_CAN_ID,IntakeConstants::RIGHT_SOLENOID_REVERSE_PCM_CAN_ID} {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    intakeTable = nt::NetworkTableInstance::GetDefault().GetTable("Intake");
    intakeTable->GetEntry("Intake Speed").SetDouble(0.0);
}

void Intake::init() {
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor.SetInverted(false);
    
}

void Intake::setControllers(frc::XboxController* controllerO, frc::XboxController* controllerD) {
    operatorController = controllerO;
    driverController = controllerD;
}

void Intake::setDefaultState() {
    state.intakeState = IntakeState::DISABLED;
    resetState();
}

void Intake::resetState() {

}

void Intake::assessInputs() {

    if (!operatorController) {
        return;
    }

    if (driverController->GetAButton() || operatorController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.altState == true;
        state.power = intakeTable->GetEntry("Intake Speed").GetDouble(0.0);
    } else {
        state.altState == false;
    }

    if (operatorController->GetAButton()) {
        state.intakeState = IntakeState::DEPLOY;
    } else {
        if (operatorController->GetBButton()) {
            state.intakeState = IntakeState::RETRACT;
        }
    }
}

void Intake::assignOutputs() {
    state.intakeState == IntakeState::RETRACT ? frc::SmartDashboard::PutString("State", "Retract") : state.intakeState == IntakeState::DEPLOY ? frc::SmartDashboard::PutString("State", "Deploy") : frc::SmartDashboard::PutString("State", "Disabled");
    state.altState ? frc::SmartDashboard::PutString("AltState", "True") : frc::SmartDashboard::PutString("AltState", "False");

    if (state.intakeState == IntakeState::DISABLED) {
        motor.Set(0);
    } else {
        if (state.altState) {
            motor.Set(state.power);
        } else {
            motor.Set(0);
        }
    }

    if (state.intakeState == IntakeState::DISABLED) {
        leftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        leftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        if (state.intakeState == IntakeState::DEPLOY) {
            leftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
            leftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
        } else {
            if (state.intakeState == IntakeState::RETRACT) {
                leftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
                leftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
            }
        }
    }
}