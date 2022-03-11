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

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
        new TrapezoidProfile.Constraints(2 * Math.PI / 2, 10));

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
        new TrapezoidProfile.Constraints(2 * Math.PI / 2, 10));

    public RotationArms(RotationArmsIO io) {
        this.io = io;

        leftController.setTolerance(ClimberConstants.rotationPositionToleranceRad);
        rightController.setTolerance(ClimberConstants.rotationPositionToleranceRad);

    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("RotationArms", ioInputs);

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

            leftController.setGoal(ClimberConstants.stowPositionRad);
            rightController.setGoal(ClimberConstants.stowPositionRad);
        }

        Logger.getInstance().recordOutput("RotationArms/LeftAngleModDeg", Units.radiansToDegrees(leftMod));
        Logger.getInstance().recordOutput("RotationArms/RightAngleModDeg", Units.radiansToDegrees(rightMod));

        double leftOutput = leftController.calculate(leftMod);
        io.setLeftVolts(leftOutput);

        double rightOutput = rightController.calculate(rightMod);
        io.setRightVolts(rightOutput);

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
        leftController.setGoal(positionRad);
        rightController.setGoal(positionRad);
    }

    public boolean atGoal() {
        return leftController.atGoal() && rightController.atGoal();
    }


    // Commands
    public final Command waitForMove = new WaitUntilCommand(this::atGoal);
    public final Command moveToStow = new InstantCommand(() -> setDesiredPosition(ClimberConstants.stowPositionRad), this);
    public final Command moveToClimbGrab = new InstantCommand(() -> setDesiredPosition(ClimberConstants.climbGrabPositionRad), this);
    public final Command moveToIntake = new InstantCommand(() -> setDesiredPosition(ClimberConstants.intakePositionRad), this);
    public final Command moveToClimbSwing = new InstantCommand(() -> setDesiredPosition(ClimberConstants.climbSwingPositionRad), this);

    
}