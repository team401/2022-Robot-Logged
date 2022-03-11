package frc.robot.subsystems.rotationArms;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rotationArms.RotationArmsIO.RotationArmsIOInputs;

public class RotationArmsSubsystem extends SubsystemBase {

    private final RotationArmsIO io;
    private final RotationArmsIOInputs ioInputs = new RotationArmsIOInputs();

    private final ProfiledPIDController leftController = new ProfiledPIDController(0.0, 0, 0.0, 
        new TrapezoidProfile.Constraints(5, 5));

    private final ProfiledPIDController rightController = new ProfiledPIDController(0.0, 0, 0.0, 
        new TrapezoidProfile.Constraints(5, 5));

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
    
}