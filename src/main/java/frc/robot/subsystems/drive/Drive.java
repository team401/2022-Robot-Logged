// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveModuleIO.DriveIOInputs;

public class Drive extends SubsystemBase {
  private DriveIOInputs[] inputs = new DriveIOInputs[4];
  private PIDController[] rotationPIDs = new PIDController[4];

  private ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds();
  
  //Why is this not DriveModuleIOComp since it is the object that implements the DriveModuleIO Interface? -Béla
  private final DriveModuleIO[] moduleIOs;
  
  public Drive(DriveModuleIO[] moduleIOs) {
      for (int i = 0; i < 4; i++) {
        inputs[i] = new DriveIOInputs();
        rotationPIDs[i] = new PIDController(DriveConstants.rotationKp.get(), 0, DriveConstants.rotationKd.get());
        rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
      }

      this.moduleIOs = moduleIOs;

      for (DriveModuleIO module : moduleIOs) {
          module.zeroEncoders();
      }
  }

  @Override
  public void periodic() {
    // Filling out data object to be sent to logger  
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("Drive" + i, inputs[i]);
    }

    //Bitwise OR does not short circuit 
    if (DriveConstants.rotationKp.hasChanged() | DriveConstants.rotationKd.hasChanged()) {
      for (PIDController c : rotationPIDs) {
        c.setP(DriveConstants.rotationKp.get());
        c.setD(DriveConstants.rotationKd.get());
      }
    }


    // Read module rotations as Rotation2ds


    // Update controllers to reach goal chassis speed
    SwerveModuleState[] moduleStates = DriveConstants.kinematics.toSwerveModuleStates(goalChassisSpeeds);
    
    for (int i = 0; i < 4; i++) {
      Rotation2d moduleRotation = new Rotation2d(inputs[i].rotationPositionRad);
      double rotationSetpointRadians = moduleStates[i].angle.getRadians();
      Logger.getInstance().recordOutput("Drive" + i + "/RotationSetpointRad", rotationSetpointRadians);
      double rotationVoltage = rotationPIDs[i].calculate(rotationSetpointRadians, moduleRotation.getRadians());
      moduleIOs[i].setRotationVoltage(rotationVoltage);
    }
      
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {

    goalChassisSpeeds = speeds;

  }
  
}
