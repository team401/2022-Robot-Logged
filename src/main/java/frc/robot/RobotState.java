package frc.robot;

import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseHistory;

public class RobotState {

    private static RobotState instance;

    //If there is no RobotState instance already, make one
    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private Pose2d fieldToBoot = new Pose2d();
    private final PoseHistory bootToVehicle = new PoseHistory(100);
    private final PoseHistory turretFixedToTurret = new PoseHistory(100);
    private ChassisSpeeds vehicleVelocity = new ChassisSpeeds();
    private double turretVelocityRadPerSec = 0.0;

    private Translation2d latestMeasuredFieldToTarget = Constants.FieldConstants.hubCenter;

    private RobotState() {
        bootToVehicle.insert(0.0, new Pose2d());
        turretFixedToTurret.insert(0.0, new Pose2d());
    }

    private static double[] poseToDoubleArray(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    public void logRobotState() {
        Logger.getInstance().recordOutput("RobotState/FieldToVehicle", poseToDoubleArray(getLatestFieldToVehicle()));
    }

    /**
     * Sets the robot pose to the given pose
     * 
     * @param fieldToVehicle The robot pose to set
     */
    public void forceRobotPose(Pose2d fieldToVehicle) {
        Pose2d bootToVehicle = this.bootToVehicle.getLatest().orElseThrow().getPose();
        Pose2d vehicleToBoot = GeomUtil.poseInverse(bootToVehicle);
        fieldToBoot = fieldToVehicle.transformBy(GeomUtil.poseToTransform(vehicleToBoot));
    }

    public void recordOdometryObservations(Pose2d bootToVehicle, ChassisSpeeds velocity) {
        this.bootToVehicle.insert(Timer.getFPGATimestamp(), bootToVehicle);
        vehicleVelocity = velocity;
    }

    public void recordTurretObservations(Rotation2d turretFixedToTurretRotation, double turretVelocityRadPerSec) {
        this.turretVelocityRadPerSec = turretVelocityRadPerSec;
        turretFixedToTurret.insert(Timer.getFPGATimestamp(), GeomUtil.poseFromRotation(turretFixedToTurretRotation));
    }

    public void recordVisionObservations(double captureTimestamp, Translation2d cameraToTarget) {
        latestMeasuredFieldToTarget = getFieldToTurret(captureTimestamp)
                .transformBy(GeomUtil.poseToTransform(Constants.VisionConstants.turretToCamera))
                .transformBy(GeomUtil.transformFromTranslation(cameraToTarget)).getTranslation();
    }

    public Pose2d getFieldToVehicle(double timestamp) {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.get(timestamp).orElseThrow());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getLatestFieldToVehicle() {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.getLatest().orElseThrow().getPose());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getPredictedFieldToVehicle(double lookaheadTime, double angularLookaheadTime) {
        return getLatestFieldToVehicle().exp(
                new Twist2d(vehicleVelocity.vxMetersPerSecond * lookaheadTime,
                        vehicleVelocity.vyMetersPerSecond * lookaheadTime,
                        vehicleVelocity.omegaRadiansPerSecond * angularLookaheadTime));
    }

    public Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(GeomUtil.poseToTransform(Constants.TurretConstants.vehicleToTurretFixed)).transformBy(GeomUtil.poseToTransform(turretFixedToTurret.get(timestamp).orElseThrow()));
    }

    public Pose2d getLatestFieldToTurret() {
        return getLatestFieldToVehicle().transformBy(GeomUtil.poseToTransform(Constants.TurretConstants.vehicleToTurretFixed)).transformBy(GeomUtil.poseToTransform(turretFixedToTurret.getLatest().orElseThrow().getPose()));
    }

    public AimingParameters getAimingParameters() {
        Pose2d fieldToPredictedVehicle = getPredictedFieldToVehicle(Constants.VisionConstants.targetingLookaheadS.get(), Constants.VisionConstants.targetingAngularLookaheadS.get());
        Pose2d fieldToPredictedTurretFixed = fieldToPredictedVehicle
            .transformBy(GeomUtil.poseToTransform(Constants.TurretConstants.vehicleToTurretFixed));
            
        Translation2d turretFixedToTargetTranslation = GeomUtil.poseInverse(fieldToPredictedTurretFixed)
            .transformBy(GeomUtil.transformFromTranslation(latestMeasuredFieldToTarget)).getTranslation();

        Translation2d vehicleToTargetTranslation = GeomUtil.poseInverse(fieldToPredictedVehicle)
            .transformBy(GeomUtil.transformFromTranslation(latestMeasuredFieldToTarget)).getTranslation();

        Rotation2d vehicleToGoalDirection = GeomUtil.direction(vehicleToTargetTranslation);

        Rotation2d turretDirection = GeomUtil.direction(turretFixedToTargetTranslation);
        double targetDistance = turretFixedToTargetTranslation.getNorm();

        Logger.getInstance().recordOutput("RobotState/TargetDistance", targetDistance);



        double feedVelocity = vehicleVelocity.vxMetersPerSecond * vehicleToGoalDirection.getSin() / targetDistance - vehicleVelocity.vyMetersPerSecond * vehicleToGoalDirection.getCos() / targetDistance - vehicleVelocity.omegaRadiansPerSecond;

        return new AimingParameters(turretDirection, targetDistance, feedVelocity);
    }

    public final class AimingParameters {
        
        //Angle of turret on robot
        private final Rotation2d turretAngle;
        //Distance in meters from target
        private final double distanceM;
        private final double velocityRadiansPerSec;

        public AimingParameters(Rotation2d turretAngle, double distanceM, double velocity) {
            this.turretAngle = turretAngle;
            this.distanceM = distanceM;
            this.velocityRadiansPerSec = velocity;
        }

        public Rotation2d getTurretAngle() {
            return turretAngle;
        }
        
        public double getDistanceM() {
            return distanceM;            
        }

        public double getVelocityRadPerSec() {
            return velocityRadiansPerSec;            
        }


    }

}