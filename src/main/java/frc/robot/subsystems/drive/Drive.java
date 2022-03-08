// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveModuleIO.DriveIOInputs;

public class Drive extends SubsystemBase {
  private DriveIOInputs[] inputs;
  private final DriveModuleIO[] moduleIOs;
  
  public Drive(DriveModuleIO[] moduleIOs) {
      for (int i = 0; i < 4; i++) {
        inputs[i] = new DriveIOInputs();
      }

      this.moduleIOs = moduleIOs;
  }

  @Override
  public void periodic() {

    // Filling out data object to be sent to logger  
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("Drive" + i, inputs[i]);
    }    
      
  }
}
