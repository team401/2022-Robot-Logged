// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

public final class Constants {
    public static final boolean tuningMode = true;

    public static final double trackWidth = Units.inchesToMeters(19.75);
    public static final double wheelBase = Units.inchesToMeters(19.75);

    public static final class CANDevices {
        public static final int frontLeftDriveMotorID = 0;
        public static final int frontLeftRotationMotorID = 1;

        public static final int frontRightDriveMotorID = 2;
        public static final int frontRightRotationMotorID = 3;

        public static final int backLeftDriveMotorID = 4;
        public static final int backLeftRotationMotorID = 5;

        public static final int backRightDriveMotorID = 6;
        public static final int backRightRotationMotorID = 7;

        public static final int frontLeftRotationEncoderID = 10;
        public static final int frontRightRotationEncoderID = 11;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 13;

        public static final int intakeMotorID = 22; 
        public static final int conveyorMotorID = 23;
        public static final int indexMotorID = 24; 

        public static final int pigeonIMU = 20;
    }

    public static final class DIOChannels {

        public static final int topBannerPort = 0;
        public static final int bottomBannerPort = 1;

    }
    
    public static final class DriveConstants {
        public static final double driveWheelGearReduction = 6.75;
        public static final double rotationWheelGearReduction = 150.0 / 7.0;
        public static final double maxSpeedMPerS = Units.feetToMeters(15.0);
        public static final double maxAngularSpeedRadPerS = 2 * Math.PI;

        // TODO measure
        public static final double wheelRadiusM = Units.inchesToMeters(2);

        // Measured on 3/9/22 with machinist's square
        // TODO verify these with 1x1 and clamping
        public static final double frontLeftAngleOffset = 0.8605632220038447;
        public static final double frontRightAngleOffset = 5.750893973783269;
        public static final double backLeftAngleOffset = 1.2854759002481673;
        public static final double backRightAngleOffset = 4.275204455837282;

        public static final TunableNumber rotationKp = new TunableNumber("Drive/RotationKp");
        public static final TunableNumber rotationKd = new TunableNumber("Drive/RotationKd");
        public static final TunableNumber driveKp = new TunableNumber("Drive/DriveKp");
        public static final TunableNumber driveKd = new TunableNumber("Drive/DriveKd");

        public static final double driveJoystickDeadbandPercent = 0.08;
        public static final double driveMaxJerk = 200.0;

        static {
            // Tuned on 3/8/22
            rotationKp.setDefault(8.0);
            rotationKd.setDefault(0.1);

            driveKp.setDefault(0.2);
            driveKd.setDefault(2.0);
        }

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //values for front left (+, +)
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //values for front right (+, -)
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //values for back left (-, +)
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //values for back right (-, -)
            );

        public static final SimpleMotorFeedforward driveModel = new SimpleMotorFeedforward(0.184, 0.1163414634);

    }

    public static final class ClimberConstants {

        // TODO: Update value
        public static final double rotationArmGearRation = 1;

        public static final double rotationEncoderConversionFactor = (2 * Math.PI) * 4096 / rotationArmGearRation;

        public static final double intakeArmPosition = 0.75;
        public static final double climbArmPosition = 0.35;
        public static final double defaultArmPosition = -0.1;

        // measured offsets
        public static final double leftRotationOffset = -1.18;
        public static final double rightRotationOffset = 1.01;

        //vertical conversion from rotations
        public static final double linearConversion = 0.625 * Math.PI;

        public static final double maxHeightMeters = 0.971;

        public static final double climberSequencePauseSeconds = 2;

    }

}

