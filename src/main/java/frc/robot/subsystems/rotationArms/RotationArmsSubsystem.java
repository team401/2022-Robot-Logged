package frc.robot.subsystems.rotationarms;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.rotationarms.RotationArmsIO.RotationArmsIOInputs;

public class RotationArmsSubsystem extends SubsystemBase {

    private final RotationArmsIO io;
    private final RotationArmsIOInputs ioInputs = new RotationArmsIOInputs();

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
        new TrapezoidProfile.Constraints(Math.PI / 2, 10));

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
        new TrapezoidProfile.Constraints(Math.PI / 2, 10));

    private double tolerance = Units.degreesToRadians(1.5);

    private double frontBoundary = Units.degreesToRadians(58.0);
    private double backBoundary = Units.degreesToRadians(-15.0);

    public RotationArmsSubsystem(RotationArmsIO io) {
        this.io = io;

        leftController.setGoal(0);
        rightController.setGoal(0);

    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("RotationArms", ioInputs);

        if (ClimberConstants.rotationArmKp.hasChanged() || ClimberConstants.rotationArmKd.hasChanged()) {
            leftController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
            rightController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
        }

        // Reset PID controllers if the robot is not enabled
        if (DriverStation.isDisabled()) {
            leftController.reset(ioInputs.leftPositionRad);
            rightController.reset(ioInputs.rightPositionRad);
        }

        double leftMod = MathUtil.angleModulus(ioInputs.leftPositionRad);
        double rightMod = MathUtil.angleModulus(ioInputs.rightPositionRad);

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
        return Math.abs(ioInputs.leftPositionRad - leftController.getGoal().position) <= tolerance && 
                Math.abs(ioInputs.rightPositionRad - rightController.getGoal().position) <= tolerance;
    }

    public boolean withinBoundaries() {
        return (ioInputs.leftPositionRad >= backBoundary && 
                ioInputs.rightPositionRad >= backBoundary && 
                ioInputs.leftPositionRad <= frontBoundary && 
                ioInputs.rightPositionRad <= frontBoundary);
    }
    
}