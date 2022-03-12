// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class MeasureKs extends CommandBase {
  private static final double stepVolts = 0.1;
  private static final double startDelayS = 2.0;

  private DoubleSupplier velocitySupplier;
  private DoubleConsumer voltageSetter;

  private final Timer timer = new Timer();

  private double voltage = 0;

  /** Creates a new MeasureDriveKs. */
  public MeasureKs(Subsystem s, DoubleSupplier velocitySupplier, DoubleConsumer voltageSetter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s);
    this.velocitySupplier = velocitySupplier;
    this.voltageSetter = voltageSetter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    voltageSetter.accept(0);

    voltage = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelayS) {
      voltageSetter.accept(0);
    } else {
      voltage = (timer.get() - startDelayS) * stepVolts;
      voltageSetter.accept(voltage);
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
    return velocitySupplier.getAsDouble() > 1e-2;
  }
}
