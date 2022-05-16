package frc.robot.subsystems.telescopes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.GeomUtil;
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

    //private boolean homed = false;
    private boolean leftHomed = true;
    private boolean rightHomed = true;
    private final Timer leftHomeTimer = new Timer();
    private final Timer rightHomeTimer = new Timer();

    private boolean atGoalOverride = false;

    private double goalPositionRad = ClimberConstants.telescopeDefaultPositionRad;

    private boolean raised = true;

    private boolean override = true;//false
    // TODO CHANGE TO FALSE WHEN FIXED TELESCOPES

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

        leftController.setTolerance(5);
        rightController.setTolerance(5);

        leftHomeTimer.start();
        rightHomeTimer.start();
    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Telescopes", ioInputs);

        /*
        if (DriverStation.isDisabled()) {
            leftController.reset(ioInputs.leftPositionRad);
            rightController.reset(ioInputs.rightPositionRad);
        }

        if (ClimberConstants.telescopeArmKp.hasChanged() || ClimberConstants.telescopeArmKd.hasChanged()) {
            leftController.setPID(ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get());
            rightController.setPID(ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get());
        }

        Logger.getInstance().recordOutput("Telescopes/Homed", leftHomed && rightHomed);

        if (!leftHomed || !rightHomed) {
            if (DriverStation.isEnabled()) {
                // Left
                if (Math.abs(ioInputs.leftVelocityRadPerS) > ClimberConstants.telescopeHomingThresholdRadPerS) {//io.getLeftCurrentDraw() < 30){//
                    leftHomeTimer.reset();
                }
                if (leftHomeTimer.hasElapsed(ClimberConstants.homingTimeS)) {
                    leftHomed = true;
                    io.resetLeftEncoder();
                    io.setLeftVolts(0);
                    leftController.reset(ioInputs.leftPositionRad * ClimberConstants.leftTelescopeMultiplier);
                }
                else {
                    io.setLeftVolts(ClimberConstants.telescopeHomingVolts);
                }
                // Right
                if (Math.abs(ioInputs.rightVelocityRadPerS) > ClimberConstants.telescopeHomingThresholdRadPerS) {//io.getRightCurrentDraw() < 30){
                    rightHomeTimer.reset();
                }
                if (rightHomeTimer.hasElapsed(ClimberConstants.homingTimeS)) {
                    rightHomed = true;
                    io.resetRightEncoder();
                    io.setRightVolts(0);
                    rightController.reset(ioInputs.rightPositionRad * ClimberConstants.rightTelescopeMultiplier);
                }
                else {
                    io.setRightVolts(ClimberConstants.telescopeHomingVolts);
                }


            }
        } else {
            double leftOutput = leftController.calculate(ioInputs.leftPositionRad * ClimberConstants.leftTelescopeMultiplier, goalPositionRad);
            leftOutput -= leftOutput < 0 ? 0.7 : 0;
            if (!override)
                io.setLeftVolts(leftOutput);

            double rightOutput = rightController.calculate(ioInputs.rightPositionRad * ClimberConstants.rightTelescopeMultiplier, goalPositionRad);
            rightOutput -= rightOutput < 0 ? 0.7 : 0;
            if (!override)
                io.setRightVolts(rightOutput);

            Logger.getInstance().recordOutput("Telescopes/LeftOutput", leftOutput);
            Logger.getInstance().recordOutput("Telescopes/RightOutput", rightOutput);

        }

        Logger.getInstance().recordOutput("Telescopes/GoalPositionRad", goalPositionRad);
        Logger.getInstance().recordOutput("Telescopes/LeftSetpointDeg", Units.radiansToDegrees(leftController.getSetpoint().position));
        Logger.getInstance().recordOutput("Telescopes/RightSetpointDeg", Units.radiansToDegrees(rightController.getSetpoint().position));
        Logger.getInstance().recordOutput("Telescopes/LeftDeg", Units.radiansToDegrees(ioInputs.leftPositionRad * ClimberConstants.leftTelescopeMultiplier));
        Logger.getInstance().recordOutput("Telescopes/RightDeg", Units.radiansToDegrees(ioInputs.rightPositionRad * ClimberConstants.rightTelescopeMultiplier));
    
        SmartDashboard.putBoolean("Telescopes At Goal", atGoal());
        */
        
    }

    public double getLeftVelocityRadPerS() {

        return ioInputs.leftVelocityRadPerS;

    }

    public double getRightVelocityRadPerS() {

        return ioInputs.rightVelocityRadPerS;

    }

    public void resetEncoders() {
        io.resetLeftEncoder();
        io.resetRightEncoder();
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
        goalPositionRad += dt * ClimberConstants.telescopeCruiseVelocity;
        if (goalPositionRad > ClimberConstants.telescopeMaxPositionRad) goalPositionRad = ClimberConstants.telescopeMaxPositionRad;
    }

    public void jogDown() {
        goalPositionRad -= dt * ClimberConstants.telescopeCruiseVelocity;
        if (goalPositionRad < ClimberConstants.telescopeHomePositionRad) goalPositionRad = ClimberConstants.telescopeHomePositionRad;
    }

    public boolean atGoal() {
        return ((leftController.atGoal() && rightController.atGoal()) || atGoalOverride) 
            && ioInputs.leftPositionRad < 2000 && ioInputs.rightPositionRad < 2000;
    }

    public boolean passedRotationSafePosition() {
        return ioInputs.leftPositionRad <= ClimberConstants.telescopeRotationSafePositionRad &&
                ioInputs.leftPositionRad <= ClimberConstants.telescopeRotationSafePositionRad;
    }

    public void setLeftVolts(double volts) {
        io.setLeftVolts(volts);
    }

    public void setRightVolts(double volts) {
        io.setRightVolts(volts);
    }

    public void home() {
        leftHomed = false;
        rightHomed = false;
        leftHomeTimer.reset();
        leftHomeTimer.start();
        rightHomeTimer.reset();
        rightHomeTimer.start();
    }

    public void stop() {
        setDesiredPosition(ioInputs.leftPositionRad);
    }

    public void setGoalOverride(boolean override) {
        atGoalOverride = override;
    }

    public void toggleRaised() {
        raised = !raised;
        if (raised)
            setDesiredPosition(540);
        else
            setDesiredPosition(ClimberConstants.telescopeHomePositionRad);
    }

    public void setOverride(boolean o) {
        override = o;
    }

    // Commands
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
    public final Command waitForRotationSafePosition() { return new WaitUntilCommand(this::passedRotationSafePosition); }
    public final Command moveToPop() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePopAboveRungRad), this); }
    public final Command moveToFull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeMaxPositionRad), this); }
    public final Command moveToLatch() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeLatchRad), this); }
    public final Command moveToPull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePullPositionRad), this); }
    public final Command moveToSwing() {return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeSwingPositionRad), this); }

}
