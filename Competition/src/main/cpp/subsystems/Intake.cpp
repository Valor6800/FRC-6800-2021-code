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
    motor.SetInverted(true);
}

void Intake::setControllers(frc::XboxController* controllerO, frc::XboxController* controllerD) {
    operatorController = controllerO;
    driverController = controllerD;
}

void Intake::setDefaultState() {
    state.deployState = false;
    state.intakeState = IntakeState::OFF;
    resetState();
}

void Intake::resetState() {

}

void Intake::assessInputs() {

    if (!operatorController) {
        return;
    }

    if (driverController->GetBumper(frc::GenericHID::kLeftHand) || operatorController->GetBumper(frc::GenericHID::kLeftHand))
        state.intakeState = IntakeState::FORWARD;
    else if (std::abs(operatorController->GetY(frc::GenericHID::kRightHand)) > 0.05 || driverController->GetAButton())
        state.intakeState = IntakeState::REVERSE;
    else
        state.intakeState = IntakeState::OFF;

    if (operatorController->GetAButton()) {
        state.deployState = true;
    } else if (operatorController->GetBButton()) {
        state.deployState = false;
    }
}

void Intake::analyzeDashboard() {
    table->PutNumber("Intake State", state.intakeState);
    table->PutBoolean("Deploy State", state.deployState);
    state.power = table->GetNumber("Intake Speed", IntakeConstants::DEFAULT_ROLLER_SPD);

    table->PutNumber("Intake Current", motor.GetOutputCurrent());
}

void Intake::assignOutputs() {
    if (state.deployState) {
        if (state.intakeState == IntakeState::FORWARD)
            motor.Set(state.power);
        else if (state.intakeState == IntakeState::REVERSE)
            motor.Set(-state.power * 0.5);
        else
            motor.Set(0);
    } else {
        motor.Set(0);
    }
    solenoid.Set(state.deployState);
}