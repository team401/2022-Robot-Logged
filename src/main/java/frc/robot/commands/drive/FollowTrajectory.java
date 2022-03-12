// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory extends CommandBase {

    private final RobotState robotState;
    private Pose2d latestFieldToVehicle;
    private final Pose2d ending;
    private final Drive driveSubsystem;
    private final Trajectory trajectory;
    private Timer timer;

    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(DriveConstants.followTrajectoryXControllerKp.get(), 0, 
                          DriveConstants.followTrajectoryXControllerKd.get()), 
        new PIDController(DriveConstants.followTrajectoryYControllerKp.get(), 0, 
                          DriveConstants.followTrajectoryYControllerKd.get()),
        new ProfiledPIDController(DriveConstants.followTrajectoryOmegaControllerKp.get(), 0, 
                                  DriveConstants.followTrajectoryOmegaControllerKp.get(),
            new TrapezoidProfile.Constraints(0.5, 0.5)));

    public FollowTrajectory(Drive drive, RobotState robotState, Trajectory traj, Pose2d end) {
        driveSubsystem = drive;
        this.robotState = robotState;
        trajectory = traj;
        ending = end;
    }

    @Override
    public void initialize() {
        //Sets robot starting point with measurements from RobotState
        robotState.forceRobotPose(robotState.getLatestFieldToVehicle());
        controller.setTolerance(new Pose2d(0.1, 0.1, new Rotation2d(0.1)));

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {
        
        latestFieldToVehicle = robotState.getLatestFieldToVehicle();
        
        Trajectory.State desired = trajectory.sample(timer.get());

        ChassisSpeeds adjustedSpeeds = controller.calculate(
            latestFieldToVehicle, desired, ending.getRotation());

        driveSubsystem.setGoalChassisSpeeds(adjustedSpeeds);

        robotState.recordOdometryObservations(latestFieldToVehicle, adjustedSpeeds);
        robotState.logRobotState();

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        driveSubsystem.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

}