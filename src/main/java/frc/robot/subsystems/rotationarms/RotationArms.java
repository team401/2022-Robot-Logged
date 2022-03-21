package frc.robot.subsystems.rotationarms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.rotationarms.RotationArmsIO.RotationArmsIOInputs;

public class RotationArms extends SubsystemBase {
    private final RotationArmsIO io;
    private final RotationArmsIOInputs ioInputs = new RotationArmsIOInputs();

    // Speed and acceleration for regular moves
    private final TrapezoidProfile.Constraints normalConstraints = new TrapezoidProfile.Constraints(2 * Math.PI / 2, 10);
    // Speed and acceleration for slower climb moves
    private final TrapezoidProfile.Constraints climbConstraints = new TrapezoidProfile.Constraints(0.15 * Math.PI , 0.75*2);//(0.15 * Math.PI / 2, 0.75)

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
        normalConstraints);

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
        normalConstraints);

    private boolean killed = false;

    private double leftOffset = 0;
    private double rightOffset = 0;
    private int setupCycleCount = 0;

    public RotationArms(RotationArmsIO io) {
        this.io = io;

        leftController.setTolerance(ClimberConstants.rotationPositionToleranceRad);
        rightController.setTolerance(ClimberConstants.rotationPositionToleranceRad);

    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("RotationArms", ioInputs);

        // if (setupCycleCount == 20) {
        //     io.resetEncoder();
        //     leftOffset = MathUtil.angleModulus(ioInputs.leftPositionRad - ClimberConstants.leftRotationOffset);
        //     rightOffset = MathUtil.angleModulus(ioInputs.rightPositionRad - ClimberConstants.rightRotationOffset);
        //     leftController.setGoal(leftOffset);
        //     rightController.setGoal(rightOffset);
        //     setupCycleCount++;
        // }
        // else {
        //     setupCycleCount++;
        // }

        if (ClimberConstants.rotationArmKp.hasChanged() || ClimberConstants.rotationArmKd.hasChanged()) {
            leftController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
            rightController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
        }

        // Positions with modulus to be between -pi, pi.  It is important to use these in the controllers
        // to prevent the arms from trying to go through their hard stops!
        double leftMod = MathUtil.angleModulus(ioInputs.leftPositionRad);
        double rightMod = MathUtil.angleModulus(ioInputs.rightPositionRad);

        // Reset PID controllers if the robot is not enabled
        if (DriverStation.isDisabled()) {
            leftController.reset(leftMod);
            rightController.reset(rightMod);

            leftController.setGoal(ClimberConstants.intakeStowPositionRad);
            rightController.setGoal(ClimberConstants.intakeStowPositionRad);
        }

        Logger.getInstance().recordOutput("RotationArms/LeftAngleModDeg", Units.radiansToDegrees(leftMod));
        Logger.getInstance().recordOutput("RotationArms/RightAngleModDeg", Units.radiansToDegrees(rightMod));


        if (!killed) {
            double leftOutput = leftController.calculate(leftMod);
            //if (ioInputs.leftPositionRad > ClimberConstants.rotationMax && leftOutput > 0)
            //    leftOutput = 0;
            //else if (ioInputs.leftPositionRad < ClimberConstants.rotationMin && leftOutput < 0)
            //    leftOutput = 0;
            Logger.getInstance().recordOutput("RotationArms/LeftOutput", leftOutput);

            double rightOutput = rightController.calculate(rightMod);
            //if (ioInputs.rightPositionRad > ClimberConstants.rotationMax && rightOutput > 0)
            //    rightOutput = 0;
            //else if (ioInputs.rightPositionRad < ClimberConstants.rotationMin && rightOutput < 0)
            //    rightOutput = 0;
            Logger.getInstance().recordOutput("RotationArms/RightOutput", rightOutput);

            //if (Math.abs(leftController.getPositionError()) < Units.degreesToRadians(100) && Math.abs(rightController.getPositionError()) < Units.degreesToRadians(100)) {
                io.setLeftVolts(leftOutput);
                io.setRightVolts(rightOutput);
            //} else {
            //    io.setLeftVolts(0);
            //    io.setRightVolts(0);
            //    setupCycleCount = 0; // Force encoders to reset.
           // }

  
        } else {
            io.setLeftVolts(0);
            io.setRightVolts(0);
        }

        Logger.getInstance().recordOutput("RotationArms/LeftSetpointDeg", Units.radiansToDegrees(leftController.getSetpoint().position));
        Logger.getInstance().recordOutput("RotationArms/RightSetpointDeg", Units.radiansToDegrees(rightController.getSetpoint().position));
    }

    public void setLeftPercent(double percent) {
        io.setLeftVolts(percent * 12);
    }

    public void setRightPercent(double percent) {
        io.setRightVolts(percent * 12);
    }

    public void setDesiredPosition(double positionRad) {
        leftController.setConstraints(normalConstraints);
        rightController.setConstraints(normalConstraints);
        leftController.setGoal(positionRad);
        rightController.setGoal(positionRad);
    }

    public void setDesiredPositionSlow(double positionRad) {
        leftController.setConstraints(climbConstraints);
        rightController.setConstraints(climbConstraints);
        leftController.setGoal(positionRad);
        rightController.setGoal(positionRad);
    }

    public boolean atGoal() {
        return leftController.atGoal() && rightController.atGoal();
    }

    public void kill() {
        setLeftPercent(0);
        setRightPercent(0);
        killed = true;
    }

    public boolean getKilled() {
        return killed;
    }

    public double getGoal() {
        return leftController.getGoal().position;
    }


    // Commands
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
    public final Command moveToStow () { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.climbStowPositionRad), this); }
    public final Command moveToClimbGrab() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.climbGrabPositionRad), this); }
    public final Command moveToIntake() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.intakePositionRad), this); }
    public final Command moveToClimbSwing() { return new InstantCommand(() -> setDesiredPositionSlow(ClimberConstants.climbSwingPositionRad), this); }
    public final Command latchRotation() { return new InstantCommand(() -> setDesiredPositionSlow(ClimberConstants.rotationLatchRad), this); }
    // Maybe add another command for after swinging out to move them slightly back while telescoping down?
    
}