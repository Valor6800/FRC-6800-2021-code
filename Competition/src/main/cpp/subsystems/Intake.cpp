#include "subsystems/Intake.h"

Intake::Intake() : ValorSubsystem(),
                        motor{IntakeConstants::MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                        solenoid{IntakeConstants::SOLENOID_FORWARD_PCM_CAN_ID} 
                        {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Intake::init() {

    initTable("Intake");
    table->PutNumber("Intake Speed", IntakeConstants::DEFAULT_ROLLER_SPD);

    motor.RestoreFactoryDefaults();
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor.SetInverted(false);
}

void Intake::setControllers(frc::XboxController* controllerO, frc::XboxController* controllerD) {
    operatorController = controllerO;
    driverController = controllerD;
}

void Intake::setDefaultState() {
    state.deployState = DeployState::RETRACT;
    state.intakeState = false;
    resetState();
}

void Intake::resetState() {

}

void Intake::assessInputs() {

    if (!operatorController) {
        return;
    }

    state.intakeState = driverController->GetAButton() || operatorController->GetBumper(frc::GenericHID::kLeftHand);

    if (operatorController->GetAButton()) {
        state.deployState = DeployState::DEPLOY;
    } else if (operatorController->GetBButton()) {
        state.deployState = DeployState::RETRACT;
    }
}

void Intake::analyzeDashboard() {
    table->PutBoolean("Intake State", state.intakeState);
    table->PutBoolean("Deploy State", state.deployState);
    state.power = table->GetNumber("Intake Speed", IntakeConstants::DEFAULT_ROLLER_SPD);

    table->PutNumber("Intake Current", motor.GetOutputCurrent());
}

void Intake::assignOutputs() {
    motor.Set(state.intakeState ? state.power : 0);
    solenoid.Set(state.deployState == DeployState::DEPLOY);
}