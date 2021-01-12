#include "subsystems/Intake.h"

Intake::Intake() : ValorSubsystem(),
                        motor{IntakeConstants::MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        compressor{IntakeConstants::COMPRESSOR_PCM_ID},
                        solenoid{IntakeConstants::SOLENOID_FORWARD_PCM_CAN_ID,IntakeConstants::SOLENOID_REVERSE_PCM_CAN_ID} {
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
    state.deployState = DeployState::DISABLED;
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
        state.intakeState == ON;
    } else {
        state.intakeState == OFF;
    }

    if (operatorController->GetAButton()) {
        state.deployState = DeployState::DEPLOY;
    } else if (operatorController->GetBButton()) {
        state.deployState = DeployState::RETRACT;
    }
}

void Intake::assignOutputs() {
    state.deployState == DeployState::RETRACT ? frc::SmartDashboard::PutString("Deploy State", "Retract") : state.deployState == DeployState::DEPLOY ? frc::SmartDashboard::PutString("Deploy State", "Deploy") : frc::SmartDashboard::PutString("Deploy State", "Disabled");
    state.intakeState == IntakeState::ON ? frc::SmartDashboard::PutString("Intake State", "On") : state.intakeState == IntakeState::OFF ? frc::SmartDashboard::PutString("Intake State", "Off") : frc::SmartDashboard::PutString("Intake State", "Disabled");
    state.power = intakeTable->GetEntry("Intake Speed").GetDouble(0.0);

    if (state.intakeState == IntakeState::DISABLED) {
        motor.Set(0);
    } else {
        if (state.intakeState == ON) {
            motor.Set(state.power);
        } else {
            motor.Set(0);
        }
    }

    if (state.deployState == DeployState::DISABLED) {
        solenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        if (state.deployState == DeployState::DEPLOY) {
            solenoid.Set(frc::DoubleSolenoid::Value::kForward);
        } else {
                solenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        }
    }
}