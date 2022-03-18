// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.GeomUtil;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
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

        public static final int leftShooterMotorID = 8;
        public static final int rightShooterMotorID = 9;

        public static final int frontLeftRotationEncoderID = 10;
        public static final int frontRightRotationEncoderID = 11;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 13;

        public static final int leftRotationMotorID = 14;
        public static final int rightRotationMotorID = 15;
        public static final int leftTelescopingMotorID = 16;
        public static final int rightTelescopingMotorID = 17;

        public static final int turretMotorID = 18;
        public static final int hoodMotorID = 19;
        public static final int turretEncoderID = 30;

        public static final int intakeMotorID = 22; 
        public static final int conveyorMotorID = 23;
        public static final int indexMotorID = 24; 

        public static final int pigeonIMU = 20;
    }

    public static final class DIOChannels {

        public static final int topBannerPort = 1;

        public static final int leftRotationArmEncoder = 2;
        public static final int rightRotationArmEncoder = 3;

        public static final int turretEncoderPulse = 4;
        public static final int turretEncoderA = 5;
        public static final int turretEncoderB = 6;

    }
    
    public static final class DriveConstants {
        public static final double driveWheelGearReduction = 6.75;
        public static final double rotationWheelGearReduction = 150.0 / 7.0;
        public static final double maxSpeedMPerS = Units.feetToMeters(15.0);
        public static final double maxAngularSpeedRadPerS = 2 * Math.PI;

        public static final double wheelRadiusM = Units.inchesToMeters(3.9028) / 2.0;

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

        public static final TunableNumber followTrajectoryXControllerKp = new TunableNumber("Drive/FollowTrajectoryXControllerKp");
        public static final TunableNumber followTrajectoryXControllerKd = new TunableNumber("Drive/FollowTrajectoryXControllerKd");

        public static final TunableNumber followTrajectoryYControllerKp = new TunableNumber("Drive/FollowTrajectoryYControllerKp");
        public static final TunableNumber followTrajectoryYControllerKd = new TunableNumber("Drive/FollowTrajectoryYControllerKd");

        public static final TunableNumber followTrajectoryThetaControllerKp = new TunableNumber("Drive/FollowTrajectoryThetaControllerKp");
        public static final TunableNumber followTrajectoryThetaControllerKd = new TunableNumber("Drive/FollowTrajectoryThetaControllerKd");

        static {
                followTrajectoryXControllerKp.setDefault(0.25);
                followTrajectoryXControllerKd.setDefault(0);

                followTrajectoryYControllerKp.setDefault(0.25);
                followTrajectoryYControllerKd.setDefault(0);

                followTrajectoryThetaControllerKp.setDefault(3.0);
                followTrajectoryThetaControllerKd.setDefault(0);

        }

        public static final double driveJoystickDeadbandPercent = 0.075;
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

        public static final TunableNumber intakeVisionKP = new TunableNumber("intakeVisionKp");
        public static final TunableNumber intakeVisionKD = new TunableNumber("intakeVisionKd");

    }

    public static final class ClimberConstants {
        public static final double rotationPositionToleranceRad = Units.degreesToRadians(5.0);

        public static final double climbStowPositionRad = Units.degreesToRadians(-10);
        public static final double intakeStowPositionRad = Units.degreesToRadians(-1);
        public static final double climbGrabPositionRad = Units.degreesToRadians(2);
        public static final double intakePositionRad = Units.degreesToRadians(25);
        public static final double climbSwingPositionRad = Units.degreesToRadians(30);
        public static final double rotationLatchRad = Units.degreesToRadians(25);

        public static final double telescopeHomePositionRad = Units.degreesToRadians(200);
        public static final double telescopePullPositionRad = Units.degreesToRadians(50);
        public static final double telescopeMaxPositionRad = Units.rotationsToRadians(14);
        public static final double telescopePopAboveRungRad = Units.degreesToRadians(700);
        public static final double telescopeLatchRad = Units.degreesToRadians(4750);
        public static final double telescopeRotationSafePositionRad = Units.degreesToRadians(2500);


        // Multipliers applied to encoders to account for inconsistent spooling
        public static final double leftTelescopeMultiplier = 0.95;
        public static final double rightTelescopeMultiplier = 1.0;


        public static final double telescopeCruiseVelocity = 4 * 2 * Math.PI;
        public static final double telescopeAcceleration = telescopeCruiseVelocity * 4;

        // measured offsets
        public static final double leftRotationOffset = 0.41786360261458094;
        public static final double rightRotationOffset = -0.7620011828295176;

        //vertical conversion from rotations
        public static final double linearConversion = 0.625 * Math.PI;

        public static final double maxHeightMeters = 0.971;

        public static final double climberSequencePauseSeconds = 2;

        //Tunable PD Numbers
        public static TunableNumber rotationArmKp = new TunableNumber("RotationArm/Kp");
        public static TunableNumber rotationArmKd = new TunableNumber("RotationArm/Kd");
        public static TunableNumber telescopeArmKp = new TunableNumber("TelescopeArm/Kp");
        public static TunableNumber telescopeArmKd = new TunableNumber("TelescopeArm/Kd");

        public static final double telescopeHomingThresholdRadPerS = Units.degreesToRadians(10);
        public static final double telescopeHomingTimeS = 0.5;
        public static final double telescopeHomingVolts = -4;

        static {
            rotationArmKp.setDefault(25.0);
            rotationArmKd.setDefault(0.5);
            telescopeArmKp.setDefault(5.0);
            telescopeArmKd.setDefault(0.0);
        }

    }

    public static final class ShooterConstants {
        public static final double hoodRackRatio = 5.23 * (458.0 / 30.0);
        public static final double hoodOffsetRad = Math.atan(2.336 / 9.800); // From CAD

        public static final double hoodHomingThresholdRadPerS = Units.degreesToRadians(10);
        public static final double hoodHomingTimeS = 0.5;
        public static final double hoodHomingVolts = -4;

        public static final TunableNumber hoodKp = new TunableNumber("Shooter/HoodKp");
        public static final TunableNumber hoodKd = new TunableNumber("Shooter/HoodKd");
        public static final TunableNumber flywheelKp = new TunableNumber("Shooter/FlywheelKp");
        public static final TunableNumber flywheelKd = new TunableNumber("Shooter/FlywheelKd");

        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelLookup = new InterpolatingTreeMap<>();
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodLookup = new InterpolatingTreeMap<>();

        public static final SimpleMotorFeedforward flywheelModel = new SimpleMotorFeedforward(0.0539, 0.0190538);
        
        public static final double hoodMinRad = 0.27;
        public static final double hoodMaxRad = 0.63;
        
        public static final double maxDistanceToTargetIn = 14*12;

        static {
            hoodKp.setDefault(0.7);
            hoodKd.setDefault(0);
            flywheelKp.setDefault(0.07);
            flywheelKd.setDefault(4);

            //TODO: redo values
            // Value is in real outputs/robot state/distance
            // Left is distance in meters, right flywheel RPM/hood position
            // hood max - 0.63rad, hood min - 0.27rad
            flywheelLookup.put(new InterpolatingDouble(3.332), new InterpolatingDouble(2250.0));
            flywheelLookup.put(new InterpolatingDouble(2.58), new InterpolatingDouble(2250.0));
            flywheelLookup.put(new InterpolatingDouble(1.81), new InterpolatingDouble(2250.0));
            flywheelLookup.put(new InterpolatingDouble(1.54), new InterpolatingDouble(2400.0));

            hoodLookup.put(new InterpolatingDouble(3.332), new InterpolatingDouble(0.63));
            hoodLookup.put(new InterpolatingDouble(2.58), new InterpolatingDouble(0.4));
            hoodLookup.put(new InterpolatingDouble(1.81), new InterpolatingDouble(0.29));
            hoodLookup.put(new InterpolatingDouble(1.54), new InterpolatingDouble(0.27));
        }



    }

    public static final class BallConstants {
        public static final double intakePower = 1.0;
        public static final double towerPower = 0.5;
    }

    public static final class TurretConstants {
        public static final double turretGearRatio = 1.0;

        public static final Pose2d vehicleToTurretFixed = GeomUtil.inchesToMeters(new Pose2d(-5.25, 0.0, Rotation2d.fromDegrees(180)));

        public static final double turretEncoderOffsetRad = -0.23344302303378667;

        public static final double turretLimitLower = -Math.PI / 2.0;
        public static final double turretLimitUpper = Math.PI / 2.0;


        public static final TunableNumber velocityKp = new TunableNumber("Turret/VelKp");
        public static final TunableNumber velocityKd = new TunableNumber("Turret/VelKd");
        public static final TunableNumber positionKp = new TunableNumber("Turret/PosKp");
        public static final TunableNumber positionKd = new TunableNumber("Turret/PosKd");

        static {
            velocityKp.setDefault(0);
            velocityKd.setDefault(0);
            positionKp.setDefault(45);
            positionKd.setDefault(0.2);
        }

        public static final SimpleMotorFeedforward turretModel = new SimpleMotorFeedforward(0.204, 2.20697674);
    }

    public static final class VisionConstants {
        public static final double widthPixels = 960;
        public static final double heightPixels = 720;
        public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
        public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);
        public static final double cameraHeightM = Units.inchesToMeters(26.517);
        public static final Pose2d turretToCamera = GeomUtil.inchesToMeters(new Pose2d(6.461, 0.0, new Rotation2d()));

        public static final TunableNumber targetingLookaheadS = new TunableNumber("Targeting/LookaheadS");
        public static final TunableNumber targetingAngularLookaheadS = new TunableNumber("Targeting/AngularLookaheadS");

        public static final TunableNumber floorToCameraAngleDeg = new TunableNumber("Vision/FloorToCameraDeg");

        static {
            floorToCameraAngleDeg.setDefault(51.0);
            targetingLookaheadS.setDefault(0.7);
            targetingAngularLookaheadS.setDefault(0.15);
        }

        // Vision target
        public static final double visionTargetDiameter =
                Units.inchesToMeters(4.0 * 12.0 + 5.375);
        public static final double visionTargetHeightLower =
                Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
        public static final double visionTargetHeightUpper =
                visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape


        static {
            floorToCameraAngleDeg.setDefault(45.0);
        }
    }

    public static final class FieldConstants {
        // Field dimensions
        public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
        public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
        public static final double hangarLength = Units.inchesToMeters(128.75);
        public static final double hangarWidth = Units.inchesToMeters(116.0);

        // Vision target
        public static final double visionTargetDiameter =
                Units.inchesToMeters(4.0 * 12.0 + 5.375);
        public static final double visionTargetHeightLower =
                Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
        public static final double visionTargetHeightUpper =
                visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape

        // Dimensions of hub and tarmac
        public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
        public static final Translation2d hubCenter =
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
        public static final double tarmacInnerDiameter = Units.inchesToMeters(219.25);
        public static final double tarmacOuterDiameter = Units.inchesToMeters(237.31);
        public static final double tarmacFenderToTip = Units.inchesToMeters(84.75);
        public static final double tarmacFullSideLength =
                tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
        public static final double tarmacMarkedSideLength =
                Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
        public static final double tarmacMissingSideLength =
                tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff
        public static final double hubSquareLength =
                tarmacOuterDiameter - (tarmacFenderToTip * 2.0);

        // Reference rotations (angle from hub to each reference point and fender side)
        public static final Rotation2d referenceARotation =
                Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
                        .plus(Rotation2d.fromDegrees(360.0 / 16.0));
        public static final Rotation2d referenceBRotation =
                referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
        public static final Rotation2d referenceCRotation =
                referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
        public static final Rotation2d referenceDRotation =
                referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
        public static final Rotation2d fenderARotation =
                referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0));
        public static final Rotation2d fenderBRotation =
                fenderARotation.rotateBy(Rotation2d.fromDegrees(90.0));
        public static final Rotation2d fenderCRotation =
                fenderBRotation.rotateBy(Rotation2d.fromDegrees(90.0));
        public static final Rotation2d fenderDRotation =
                fenderCRotation.rotateBy(Rotation2d.fromDegrees(90.0));

        // Reference points (centered of the sides of the tarmac if they formed a complete octagon, plus
        // edges of fender)
        public static final Pose2d referenceA =
                new Pose2d(hubCenter, referenceARotation).transformBy(
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
        public static final Pose2d referenceB =
                new Pose2d(hubCenter, referenceBRotation).transformBy(
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
        public static final Pose2d referenceC =
                new Pose2d(hubCenter, referenceCRotation).transformBy(
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
        public static final Pose2d referenceD =
                new Pose2d(hubCenter, referenceDRotation).transformBy(
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
        public static final Pose2d fenderA =
                new Pose2d(hubCenter, fenderARotation).transformBy(
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
        public static final Pose2d fenderB =
                new Pose2d(hubCenter, fenderBRotation).transformBy(
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
        public static final Pose2d fenderC =
                new Pose2d(hubCenter, fenderCRotation).transformBy(
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
        public static final Pose2d fenderD =
                new Pose2d(hubCenter, fenderDRotation).transformBy(
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));

        // Cargo points
        public static final double cornerToCargoY = Units.inchesToMeters(15.56);
        public static final double referenceToCargoY =
                (tarmacFullSideLength / 2.0) - cornerToCargoY;
        public static final double referenceToCargoX = Units.inchesToMeters(40.44);
        public static final Pose2d cargoA = referenceA.transformBy(
                GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
        public static final Pose2d cargoB = referenceA.transformBy(
                GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
        public static final Pose2d cargoC = referenceB.transformBy(
                GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
        public static final Pose2d cargoD = referenceC.transformBy(
                GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
        public static final Pose2d cargoE = referenceD.transformBy(
                GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
        public static final Pose2d cargoF = referenceD.transformBy(
                GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));

        // Terminal cargo point
        public static final Rotation2d terminalOuterRotation =
                Rotation2d.fromDegrees(133.75);
        public static final double terminalLength =
                Units.inchesToMeters(324.0 - 256.42);
        public static final double terminalWidth = Math.tan(
                Rotation2d.fromDegrees(180.0).minus(terminalOuterRotation).getRadians())
                * terminalLength;
        public static final Pose2d terminalCenter =
                new Pose2d(new Translation2d(terminalLength / 2.0, terminalWidth / 2.0),
                        terminalOuterRotation.minus(Rotation2d.fromDegrees(90.0)));
        public static final double terminalCargoOffset = Units.inchesToMeters(10.43);
        public static final Pose2d cargoG = terminalCenter
                .transformBy(GeomUtil.transformFromTranslation(terminalCargoOffset, 0.0));
    }

    public static final class AutoConstants {
        public static final double kMaxVelocityMetersPerSecond = 0.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;
    }

}

