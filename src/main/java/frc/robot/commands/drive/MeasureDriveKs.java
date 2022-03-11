// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class MeasureDriveKs extends CommandBase {
  private static final double stepVolts = 0.1;
  private static final double startDelayS = 2.0;

  private final Drive drive;

  private final Timer timer = new Timer();

  private double voltage = 0;

  /** Creates a new MeasureDriveKs. */
  public MeasureDriveKs(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();
    

    SwerveModuleState zero = new SwerveModuleState(0, new Rotation2d());
    drive.setGoalModuleStates(new SwerveModuleState[] {zero, zero, zero, zero});

    voltage = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelayS) {
      drive.setDriveVoltages(new double[] {0, 0, 0, 0});
    } else {
      voltage = (timer.get() - startDelayS) * stepVolts;
      drive.setDriveVoltages(new double[] {voltage, voltage, voltage, voltage});
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("KS: " + voltage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getAverageSpeedRadPerS() > 1e-2;
  }
}
