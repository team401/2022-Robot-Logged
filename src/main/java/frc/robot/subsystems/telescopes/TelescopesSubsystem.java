package frc.robot.subsystems.telescopes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final double dt = 0.02;

    private boolean atGoalOverride = false;

    private double goalPositionM = ClimberConstants.telescopeDefaultPositionM;

    private boolean override = false;

    private final ProfiledPIDController leftController = new ProfiledPIDController(
            ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get(),
            new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM));

    private final ProfiledPIDController rightController = new ProfiledPIDController(
            ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get(),
            new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM));

    public TelescopesSubsystem(TelescopesIO io) {
        this.io = io;

        leftController.setGoal(ClimberConstants.telescopeHomePositionM);
        rightController.setGoal(ClimberConstants.telescopeHomePositionM);

        leftController.setTolerance(ClimberConstants.telescopeGoalToleranceM);
        rightController.setTolerance(ClimberConstants.telescopeGoalToleranceM);
    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Telescopes", ioInputs);

        double leftOutput = 0;
        double rightOutput = 0;

        // PID Changes
        if (ClimberConstants.telescopeArmKp.hasChanged() || ClimberConstants.telescopeArmKd.hasChanged()) {
            leftController.setPID(ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get());
            rightController.setPID(ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get());
        }

        if (DriverStation.isEnabled()){
            if (io.isLeftLidarOnline()) {
                double output = leftController.calculate(ioInputs.leftPositionM, goalPositionM);
                output -= output < 0 ? 0.7 : 0;
                leftOutput = override ? leftOutput : output;
            }
            if (io.isRightLidarOnline()) {
                double output = rightController.calculate(ioInputs.rightPositionM, goalPositionM);
                output -= output < 0 ? 0.7 : 0;
                rightOutput = override ? rightOutput : output;
            }
        }

        io.setLeftVolts(leftOutput);
        io.setRightVolts(rightOutput);

        Logger.getInstance().recordOutput("Telescopes/LeftM", ioInputs.leftPositionM);
        Logger.getInstance().recordOutput("Telescopes/RightM", ioInputs.rightPositionM);
        Logger.getInstance().recordOutput("Telescopes/GoalPositionM", goalPositionM);
        Logger.getInstance().recordOutput("Telescopes/LeftControllerSetpointM", leftController.getSetpoint().position);
        Logger.getInstance().recordOutput("Telescopes/RightControllerSetpointM", rightController.getSetpoint().position);
        Logger.getInstance().recordOutput("Telescopes/AtGoal", atGoal());
        Logger.getInstance().recordOutput("Telescopes/AtGoalOverride", atGoalOverride);
        Logger.getInstance().recordOutput("Telescopes/LeftOutput", leftOutput);
        Logger.getInstance().recordOutput("Telescopes/RightOutput", rightOutput);
        Logger.getInstance().recordOutput("Telescopes/Override", override);
        SmartDashboard.putBoolean("Telescopes At Goal", atGoal());

    }

    public void setLeftPercent(double percent) {
        io.setLeftVolts(percent * 12);
    }

    public void setRightPercent(double percent) {
        io.setRightVolts(percent * 12);
    }

    public void setDesiredPosition(double positionM) {
        goalPositionM = positionM;
    }

    public void jogUp() {
        goalPositionM += dt * ClimberConstants.telescopeCruiseVelocityM;
        if (goalPositionM > ClimberConstants.telescopeMaxPositionM) goalPositionM = ClimberConstants.telescopeMaxPositionM;
    }

    public void jogDown() {
        goalPositionM -= dt * ClimberConstants.telescopeCruiseVelocityM;
        if (goalPositionM < ClimberConstants.telescopeHomePositionM) goalPositionM = ClimberConstants.telescopeHomePositionM;
    }

    public boolean atGoal() {
        return (leftController.atGoal() && rightController.atGoal()) || atGoalOverride;
    }

    public boolean passedRotationSafePosition() {
        return ioInputs.leftPositionM <= ClimberConstants.telescopeRotationSafePositionM &&
                ioInputs.rightPositionM <= ClimberConstants.telescopeRotationSafePositionM;
    }

    public void setLeftVolts(double volts) {
        io.setLeftVolts(volts);
    }

    public void setRightVolts(double volts) {
        io.setRightVolts(volts);
    }

    public void stop() {
        setDesiredPosition(ioInputs.rightPositionM);
    }

    public void setGoalOverride(boolean override) {
        atGoalOverride = override;
    }

    public void setOverride(boolean o) {
        override = o;
    }

    // Commands
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
    public final Command waitForRotationSafePosition() { return new WaitUntilCommand(this::passedRotationSafePosition); }
    public final Command moveToPop() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePopAboveRungM), this); }
    public final Command moveToFull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeMaxPositionM), this); }
    public final Command moveToLatch() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeLatchM), this); }
    public final Command moveToPull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePullPositionM), this); }
    public final Command moveToSwing() {return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeSwingPositionM), this); }

}
