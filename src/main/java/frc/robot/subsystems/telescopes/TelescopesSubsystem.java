package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.telescopes.TelescopesIO.TelescopesIOInputs;

public class TelescopesSubsystem extends SubsystemBase {

    private final TelescopesIO io;
    private final TelescopesIOInputs ioInputs = new TelescopesIOInputs();

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get(), 
        new TrapezoidProfile.Constraints(5, 5));

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.telescopeArmKp.get(), 0, ClimberConstants.telescopeArmKd.get(), 
        new TrapezoidProfile.Constraints(5, 5));

    public TelescopesSubsystem(TelescopesIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Telescopes", ioInputs);

        double leftOutput = leftController.calculate(ioInputs.leftPositionIn);
        io.setLeftVolts(leftOutput);

        double rightOutput = leftController.calculate(ioInputs.rightPositionIn);
        io.setRightVolts(rightOutput);

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

    public void setDesiredPosition(double positionIn) {
        leftController.setGoal(positionIn);
        rightController.setGoal(positionIn);
    }
    
}
