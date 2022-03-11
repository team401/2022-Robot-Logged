package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.telescopes.TelescopesIO.TelescopesIOInputs;

public class TelescopesSubsystem extends SubsystemBase {

    private final TelescopesIO io;
    private final TelescopesIOInputs ioInputs = new TelescopesIOInputs();

    public TelescopesSubsystem(TelescopesIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Telescopes", ioInputs);

    }

    void resetEncoders() {
        io.resetEncoders();
    }

    void setLeftPercent(double percent) {
        io.setLeftVolts(percent * 12);
    }

    void setRightPercent(double percent) {
        io.setRightVolts(percent * 12);
    }
    
}
