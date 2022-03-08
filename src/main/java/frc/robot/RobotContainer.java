// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveModuleIO;
import frc.robot.subsystems.drive.DriveModuleIOComp;

public class RobotContainer {
  private final Drive drive;

  public RobotContainer() {
    drive = new Drive(new DriveModuleIO[] {

      new DriveModuleIOComp(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID, 
                            CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset),
                            
      new DriveModuleIOComp(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID, 
                            CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset),
                            
      new DriveModuleIOComp(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID, 
                            CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset),
                            
      new DriveModuleIOComp(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
                            CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset)
      
      }
    );
    
    configureButtonBindings();
  }
  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}