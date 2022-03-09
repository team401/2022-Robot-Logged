// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveWithJoysticks extends CommandBase {
  private final Drive drive;
  private final DoubleSupplier xPercent;
  private final DoubleSupplier yPercent;
  private final DoubleSupplier omegaPercent;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent) {
    this.drive = drive;
    this.xPercent = xPercent;
    this.yPercent = yPercent;
    this.omegaPercent = omegaPercent;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMPerS = xPercent.getAsDouble() * DriveConstants.maxSpeedMPerS;
    double yMPerS = yPercent.getAsDouble() * DriveConstants.maxSpeedMPerS;
    double omegaRadPerS = omegaPercent.getAsDouble() * DriveConstants.maxAngularSpeedRadPerS;

    // Square omega to get better resolution at lower speeds
    omegaRadPerS = Math.signum(omegaRadPerS) * (omegaRadPerS * omegaRadPerS);

    // Convert to field relative speeds
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, drive.getPose().getRotation());

    drive.setGoalChassisSpeeds(targetSpeeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
