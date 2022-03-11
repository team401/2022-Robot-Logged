// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory extends CommandBase {

    private RobotState robotState;
    private Pose2d latestFieldToVehicle;

    private Pose2d desiredPose;
    
    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(0.5, 0.5)));


    public FollowTrajectory(Drive drive, RobotState state, Pose2d pose) {

        desiredPose = pose;
        robotState = state;
        
    }

    @Override
    public void initialize() {

        //Sets robot starting point with measurements from RobotState
        robotState.forceRobotPose(robotState.getLatestFieldToVehicle());

    }

    @Override
    public void execute() {
        
        latestFieldToVehicle = robotState.getLatestFieldToVehicle();


        robotState.logRobotState();
    }



}