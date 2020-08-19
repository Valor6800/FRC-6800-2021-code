#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/units.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <adi/ADIS16448_IMU.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>

class Drivetrain2 : public ValorSubsystem {
    public:
        Drivetrain2();
        void init();
        void setController(frc::XboxController*);

        void setDefaultState();
        void assessInputs();
        void assignOutputs();

        void resetState();

        // ramsete methods
        void resetEncoders();
        void resetOdometry(frc::Pose2d pose);
        void resetIMU();

        units::meter_t getLeftDistance();
        units::meter_t getRightDistance();
        double getHeading();
        double getTurnRate();
        frc::Pose2d getPose();
        frc::DifferentialDriveWheelSpeeds getWheelSpeeds();
        void tankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts);

        enum DrivetrainState {
            DISABLED,
            AUTO,
            MANUAL
        };

        struct x {
            DrivetrainState drivetrainState;

            double rightTrigger;
            double leftTrigger;
            double leftJoystickX;
            bool Ybutton;
            bool rightBumper;
            double currentLeftPower;
            double currentRightPower;
            int limelightState;
        } state;

        frc::DifferentialDriveKinematics kDriveKinematics;
        frc::SimpleMotorFeedforward<units::meters> kSimpleMotorFeedforward;
        frc::TrajectoryConfig kTrajectoryConfigF;
        frc::TrajectoryConfig kTrajectoryConfigR;
        frc::DifferentialDriveVoltageConstraint kDifferentialDriveVoltageConstraint;

    private:
        rev::CANSparkMax leftDriveLead;
        rev::CANSparkMax leftDriveFollowA;
        rev::CANSparkMax leftDriveFollowB;
        rev::CANSparkMax rightDriveLead;
        rev::CANSparkMax rightDriveFollowA;
        rev::CANSparkMax rightDriveFollowB;

        rev::CANPIDController leftPIDController = leftDriveLead.GetPIDController();
        rev::CANPIDController rightPIDController = rightDriveLead.GetPIDController();
        rev::CANEncoder leftCANEncoder = leftDriveLead.GetEncoder();
        rev::CANEncoder rightCANEncoder = rightDriveLead.GetEncoder();

        frc::ADIS16448_IMU imu{};

        frc::DifferentialDriveOdometry odometry;

        frc::XboxController* driverController;
};