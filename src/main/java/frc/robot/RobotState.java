package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseHistory;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private Pose2d fieldToBoot = new Pose2d();
    private final PoseHistory bootToVehicle = new PoseHistory(100);
    private final PoseHistory vehicleToTurretFixed = new PoseHistory(100);
    private final PoseHistory turretFixedToTurret = new PoseHistory(100);
    private ChassisSpeeds vehicleVelocity = new ChassisSpeeds();
    private double turretVelocityRadPerSec = 0.0;

    private Pose2d latestMeasuredFieldToTarget = new Pose2d();

    private RobotState() {
        bootToVehicle.insert(0.0, new Pose2d());
        vehicleToTurretFixed.insert(0.0, new Pose2d());
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
     * @param fieldToTarget  The position of the target in the field frame to start
     *                       with. Vision will update this over time.
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

    public Pose2d getFieldToVehicle(double timestamp) {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.get(timestamp).orElseThrow());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getLatestFieldToVehicle() {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.getLatest().orElseThrow().getPose());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getPredictedFieldToVehicle(double lookaheadTime) {
        return getLatestFieldToVehicle().exp(
                new Twist2d(vehicleVelocity.vxMetersPerSecond * lookaheadTime,
                        vehicleVelocity.vyMetersPerSecond * lookaheadTime,
                        vehicleVelocity.omegaRadiansPerSecond * lookaheadTime));
    }

}