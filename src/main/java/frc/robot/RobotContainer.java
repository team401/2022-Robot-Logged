// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.ServerRuntimeException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.MeasureDriveKs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveAngleIOComp;
import frc.robot.subsystems.drive.DriveModuleIO;
import frc.robot.subsystems.drive.DriveModuleIOComp;

public class RobotContainer {
  private final Drive drive;

  private final XboxController gamepad = new XboxController(0);

  private final DriveWithJoysticks driveWithJoysticks;

  public RobotContainer() {
    // Create subsystems
    drive = new Drive(new DriveModuleIO[] {
        new DriveModuleIOComp(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID,
            CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset),
        new DriveModuleIOComp(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID,
            CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset),
        new DriveModuleIOComp(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID,
            CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset),
        new DriveModuleIOComp(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
            CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset)
    }, new DriveAngleIOComp());

    // Create commands
    driveWithJoysticks = new DriveWithJoysticks(

        drive, 
        () -> -gamepad.getLeftY(),
        () -> -gamepad.getLeftX(), 
        () -> -gamepad.getRightX()
        
    );

    // Bind default commands
    drive.setDefaultCommand(driveWithJoysticks);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    SwerveModuleState zero = new SwerveModuleState();
    SwerveModuleState[] zeros = new SwerveModuleState[] { zero, zero, zero, zero };
    return new InstantCommand(() -> drive.setGoalModuleStates(zeros), drive).andThen(new WaitCommand(2.0))
        .andThen(new InstantCommand(() -> drive.setDriveVoltages(new double[] { 4, 4, 4, 4 }), drive)).andThen(new WaitCommand(10.0));
  }
}