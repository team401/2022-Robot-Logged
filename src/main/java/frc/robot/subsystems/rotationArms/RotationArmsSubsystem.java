package frc.robot.subsystems.rotationArms;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.rotationArms.RotationArmsIO.RotationArmsIOInputs;

public class RotationArmsSubsystem extends SubsystemBase {

    private final RotationArmsIO io;
    private final RotationArmsIOInputs ioInputs = new RotationArmsIOInputs();

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
        new TrapezoidProfile.Constraints(5, 5));

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
        new TrapezoidProfile.Constraints(5, 5));

    private double tolerance = Units.degreesToRadians(1.5);

    private double frontBoundary = Units.degreesToRadians(58.0);
    private double backBoundary = Units.degreesToRadians(-15.0);

    public RotationArmsSubsystem(RotationArmsIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("RotationArms", ioInputs);

        double leftOutput = leftController.calculate(ioInputs.leftPositionRad);
        io.setLeftVolts(leftOutput);

        double rightOutput = leftController.calculate(ioInputs.rightPositionRad);
        io.setRightVolts(rightOutput);

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