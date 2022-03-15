// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveAngleIO.DriveAngleIOInputs;
import frc.robot.subsystems.drive.DriveModuleIO.DriveModuleIOInputs;

public class Drive extends SubsystemBase {
  private DriveModuleIOInputs[] inputs = new DriveModuleIOInputs[4];
  private DriveAngleIOInputs angleInputs = new DriveAngleIOInputs();
  private PIDController[] rotationPIDs = new PIDController[4];
  private SwerveModuleState[] goalModuleStates = new SwerveModuleState[4];
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d());

  // Why is this not DriveModuleIOComp since it is the object that implements the
  // DriveModuleIO Interface? -Béla
  private final DriveModuleIO[] moduleIOs;
  private final DriveAngleIO angleIO;

  // If true, modules will run velocity control from the setpoint velocities in
  // moduleStates
  // If false, modules will not run velocity control from the setpoint velocities
  // in moduleStates,
  // and module drive motors will not be commanded to do anything. This allows
  // other setters,
  // such as the "setDriveVoltages" to have control of the modules.
  private boolean velocityControlEnabled = true;

  public Drive(DriveModuleIO[] moduleIOs, DriveAngleIO angleIO) {
    for (int i = 0; i < 4; i++) {
      inputs[i] = new DriveModuleIOInputs();
      rotationPIDs[i] = new PIDController(DriveConstants.rotationKp.get(), 0, DriveConstants.rotationKd.get());
      rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
      goalModuleStates[i] = new SwerveModuleState();
    }

    this.moduleIOs = moduleIOs;
    this.angleIO = angleIO;

    for (DriveModuleIO module : moduleIOs) {
      module.zeroEncoders();
    }

    angleIO.resetHeading();
  }

  @Override
  public void periodic() {
    // Read inputs from module IO layers and tell the logger about them
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("Drive" + i, inputs[i]);
    }
    angleIO.updateInputs(angleInputs);
    Logger.getInstance().processInputs("DriveAngle", angleInputs);

    // Update odometry and report to RobotState
    Rotation2d headingRotation = new Rotation2d(MathUtil.angleModulus(angleInputs.headingRad));
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = new SwerveModuleState(inputs[i].driveVelocityRadPerS * DriveConstants.wheelRadiusM,
          new Rotation2d(MathUtil.angleModulus(inputs[i].rotationPositionRad)));
    }
    Pose2d bootToVehicle = odometry.update(headingRotation, measuredStates);
    ChassisSpeeds measuredChassis = DriveConstants.kinematics.toChassisSpeeds(measuredStates);
    RobotState.getInstance().recordOdometryObservations(bootToVehicle, measuredChassis);


    // Check if our gain tunables have changed, and if they have, update accordingly
    if (DriveConstants.rotationKp.hasChanged() || DriveConstants.rotationKd.hasChanged()) {
      for (PIDController c : rotationPIDs) {
        c.setP(DriveConstants.rotationKp.get());
        c.setD(DriveConstants.rotationKd.get());
      }
    }

    if (DriveConstants.driveKp.hasChanged() || DriveConstants.driveKd.hasChanged()) {
      for (DriveModuleIO moduleIO : moduleIOs) {
        moduleIO.setDrivePD(DriveConstants.driveKp.get(), DriveConstants.driveKd.get());
      }
    }

    // Optimize and set setpoints for each individual module
    for (int i = 0; i < 4; i++) {
      // Wrap encoder value to be within -pi, pi radians
      Rotation2d moduleRotation = new Rotation2d(MathUtil.angleModulus(inputs[i].rotationPositionRad));

      // Optimize each module state
      SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
      double rotationSetpointRadians = optimizedState.angle.getRadians();
      double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

      // Set module speed
      if (velocityControlEnabled) {
        double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
        double ffVolts = DriveConstants.driveModel.calculate(speedRadPerS);

        moduleIOs[i].setDriveVelocity(speedRadPerS, ffVolts);
        Logger.getInstance().recordOutput("Drive" + i + "/SpeedSetpointRadPerS", speedRadPerS);
      }

      // Set module rotation
      Logger.getInstance().recordOutput("Drive" + i + "/RotationSetpointRad", rotationSetpointRadians);
      double rotationVoltage = rotationPIDs[i].calculate(rotationSetpointRadians, moduleRotation.getRadians());
      Logger.getInstance().recordOutput("Drive" + i + "/RotationErrorDegrees",
          Units.radiansToDegrees(rotationPIDs[i].getPositionError()));
      moduleIOs[i].setRotationVoltage(rotationVoltage);
    }

    Logger.getInstance().recordOutput("Drive/AverageRadPerS", getAverageSpeedRadPerS());
  }

  /**
   * Sets the target module states for each module. This can be used to
   * individually control each module.
   * 
   * @param states The array of states for each module
   */
  public void setGoalModuleStates(SwerveModuleState[] states) {
    velocityControlEnabled = true;
    goalModuleStates = states;
  }

  /**
   * Sets the raw voltages of the module drive motors. Heading is still set from
   * the angles set in
   * setGoalModuleStates. Note that this method disables velocity control until
   * setModuleStates or
   * setGoalChassisSpeeds is called again.
   * 
   * The primary use of this is to characterize the drive.
   * 
   * @param voltages The array of voltages to set in the modules
   */
  public void setDriveVoltages(double[] voltages) {
    velocityControlEnabled = false;
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].setDriveVoltage(voltages[i]);
    }
  }

  /**
   * Sets the goal chassis speeds for the entire chassis.
   * 
   * @param speeds The target speed for the chassis
   */
  public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.maxSpeedMPerS);

    setGoalModuleStates(moduleStates);
  }

  /**
   * Returns the average angular speed of each wheel. This is used to characterize
   * the drive.
   * 
   * @return The average speed of each module in rad/s
   */
  public double getAverageSpeedRadPerS() {
    double sum = 0;
    for (int i = 0; i < 4; i++) {
      sum += inputs[i].driveVelocityRadPerS;
    }
    return sum / 4.0;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, new Rotation2d());
  }

}
