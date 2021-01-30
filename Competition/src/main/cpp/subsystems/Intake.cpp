#include "subsystems/Intake.h"

Intake::Intake() : ValorSubsystem(),
                        motor{IntakeConstants::MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless}
                        //solenoid{IntakeConstants::SOLENOID_FORWARD_PCM_CAN_ID,IntakeConstants::SOLENOID_FORWARD_PCM_CAN_ID} 
                        {
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
    state.deployState = DeployState::RETRACT;
    state.intakeState = IntakeState::OFF;
    resetState();
}

void Intake::resetState() {

}

void Intake::assessInputs() {

    if (!operatorController) {
        return;
    }

    if (driverController->GetAButton() || operatorController->GetBumper(frc::GenericHID::kLeftHand)) {
        state.intakeState = ON;
    } else {
        state.intakeState = OFF;
    }

    if (operatorController->GetAButton()) {
        state.deployState = DeployState::DEPLOY;
    } else if (operatorController->GetBButton()) {
        state.deployState = DeployState::RETRACT;
    }
}

void Intake::assignOutputs() {
    state.deployState == DeployState::RETRACT ? frc::SmartDashboard::PutString("Deploy State", "Retract") : frc::SmartDashboard::PutString("Deploy State", "Deploy");
    state.intakeState == IntakeState::ON ? frc::SmartDashboard::PutString("Intake State", "On") : frc::SmartDashboard::PutString("Intake State", "Off");
    state.power = intakeTable->GetEntry("Intake Speed").GetDouble(0.0);

    if (state.intakeState == ON) {
        motor.Set(state.power);
    } else {
        motor.Set(0);
    }


    //need to veriy implemetation of single solenoid - you had double before
    if (state.deployState == DeployState::DEPLOY) {
        //solenoid.Set(frc::Solenoid::Value::kForward);
    } else {
            ///solenoid.Set(frc::Solenoid::Value::kReverse);
    }

}