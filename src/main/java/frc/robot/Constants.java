// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

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
    }
    
    public static final class DriveConstants {

        public static final double driveWheelGearReduction = 6.75;
        public static final double rotationWheelGearReduction = 150.0 / 7.0;

        public static final double frontLeftAngleOffset = Units.degreesToRadians(0);
        public static final double frontRightAngleOffset = Units.degreesToRadians(0);
        public static final double backLeftAngleOffset = Units.degreesToRadians(0);
        public static final double backRightAngleOffset = Units.degreesToRadians(0);

    }

}

