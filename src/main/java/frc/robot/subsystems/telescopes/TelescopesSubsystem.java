package frc.robot.subsystems.telescopes;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.telescopes.TelescopesIO.TelescopesIOInputs;

public class TelescopesSubsystem extends SubsystemBase {
    private final TelescopesIO io;
    private final TelescopesIOInputs ioInputs = new TelescopesIOInputs();

    private boolean homed = false;
    private final Timer homeTimer = new Timer();

    private double goalPositionRad = ClimberConstants.telescopeHomePositionRad;

    private final ProfiledPIDController leftController = new ProfiledPIDController(
            ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get(),
            new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocity, ClimberConstants.telescopeAcceleration));

    private final ProfiledPIDController rightController = new ProfiledPIDController(
            ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get(),
            new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocity, ClimberConstants.telescopeAcceleration));

    public TelescopesSubsystem(TelescopesIO io) {
        this.io = io;

        leftController.setGoal(ClimberConstants.telescopeHomePositionRad);
        rightController.setGoal(ClimberConstants.telescopeHomePositionRad);

        leftController.setTolerance(Units.degreesToRadians(50.0));
        rightController.setTolerance(Units.degreesToRadians(50.0));
    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Telescopes", ioInputs);

        if (DriverStation.isDisabled()) {
            leftController.reset(ioInputs.leftPositionRad);
            rightController.reset(ioInputs.rightPositionRad);
        }

        if (ClimberConstants.telescopeArmKp.hasChanged() || ClimberConstants.telescopeArmKd.hasChanged()) {
            leftController.setPID(ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get());
            rightController.setPID(ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get());
        }


        if (!homed) {
            if (DriverStation.isEnabled()) {
                if (Math.abs(ioInputs.leftVelocityRadPerS) < ClimberConstants.telescopeHomingThresholdRadPerS
                        && Math.abs(ioInputs.rightVelocityRadPerS) < ClimberConstants.telescopeHomingThresholdRadPerS) {
                    homeTimer.start();
                } else {
                    homeTimer.stop();
                    homeTimer.reset();
                }

                if (homeTimer.hasElapsed(ClimberConstants.telescopeHomingTimeS)) {
                    homed = true;
                    io.resetEncoders();
                    io.setLeftVolts(0);
                    io.setRightVolts(0);
                    leftController.reset(ioInputs.leftPositionRad * ClimberConstants.leftTelescopeMultiplier);
                    rightController.reset(ioInputs.rightPositionRad * ClimberConstants.rightTelescopeMultiplier);
                } else {
                    io.setLeftVolts(ClimberConstants.telescopeHomingVolts);
                    io.setRightVolts(ClimberConstants.telescopeHomingVolts);
                }
            }
        } else {
            double leftOutput = leftController.calculate(ioInputs.leftPositionRad * ClimberConstants.leftTelescopeMultiplier, goalPositionRad);
            io.setLeftVolts(leftOutput);

            double rightOutput = rightController.calculate(ioInputs.rightPositionRad * ClimberConstants.rightTelescopeMultiplier, goalPositionRad);
            io.setRightVolts(rightOutput);
        }

        Logger.getInstance().recordOutput("Telescopes/LeftSetpointDeg", Units.radiansToDegrees(leftController.getSetpoint().position));
        Logger.getInstance().recordOutput("Telescopes/RightSetpointDeg", Units.radiansToDegrees(rightController.getSetpoint().position));
        Logger.getInstance().recordOutput("Telescopes/LeftDeg", Units.radiansToDegrees(ioInputs.leftPositionRad * ClimberConstants.leftTelescopeMultiplier));
        Logger.getInstance().recordOutput("Telescopes/RightDeg", Units.radiansToDegrees(ioInputs.rightPositionRad * ClimberConstants.rightTelescopeMultiplier));
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public void setLeftPercent(double percent) {
        io.setLeftVolts(percent * 12);
    }

    public void setRightPercent(double percent) {
        io.setRightVolts(percent * 12);
    }

    public void setDesiredPosition(double positionRad) {
        goalPositionRad = positionRad;
    }

    public void jogUp() {
        goalPositionRad += 0.02 * ClimberConstants.telescopeCruiseVelocity;
        if (goalPositionRad > ClimberConstants.telescopeMaxPositionRad) goalPositionRad = ClimberConstants.telescopeMaxPositionRad;
    }

    public void jogDown() {
        goalPositionRad -= 0.02 * ClimberConstants.telescopeCruiseVelocity;
        if (goalPositionRad < ClimberConstants.telescopeHomePositionRad) goalPositionRad = ClimberConstants.telescopeHomePositionRad;
    }

    public boolean atGoal() {
        return leftController.atGoal() && rightController.atGoal();
    }

    // Commands
    public final Command waitForMove = new WaitUntilCommand(this::atGoal);
    public final Command moveToStow = new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeHomePositionRad), this);
    public final Command moveToPop = new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePopAboveRungRad), this);
    public final Command moveToFull = new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeMaxPositionRad), this);

}