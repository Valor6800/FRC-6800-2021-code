// #pragma once

// #include "ValorSubsystem.h"

// #include <rev/CANSparkMax.h>
// #include <frc/XboxController.h>
// #include "Constants.h"

// #ifndef DRIVETRAIN2_H
// #define DRIVETRAIN2_H

// class Drivetrain2 : public ValorSubsystem {
//     public:
//         Drivetrain2();

//         void init();
//         void setController(frc::XboxController* controller);

//         void setDefaultState();
//         void assessInputs();
//         void assignOutputs();

//         void resetState();


//     private:
//         rev::CANSparkMax leftDriveLead;
//         rev::CANSparkMax rightDriveLead;
//         rev::CANSparkMax leftDriveFollowA;
//         rev::CANSparkMax leftDriveFollowB;
//         rev::CANSparkMax rightDriveFollowA;
//         rev::CANSparkMax rightDriveFollowB;

//         // rev::CANEncoder leftEnc = leftDriveLead.GetEncoder();
//         // rev::CANEncoder rightEnc = rightDriveLead.GetEncoder();

//         // rev::CANPIDController leftPID;
//         // rev::CANPIDController rightPID;

//         frc::XboxController* driverController;

// };

// #endif