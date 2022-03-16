package frc.robot.subsystems.intakevision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakevision.IntakeVisionIO.IntakeVisionIOInputs;

public class IntakeVision extends SubsystemBase {

    private final IntakeVisionIO io;
    private final IntakeVisionIOInputs ioInputs = new IntakeVisionIOInputs();

    public IntakeVision(IntakeVisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("IntakeVision", ioInputs);

        io.setLeds(false);
    }

    public boolean hasTarget() {
        return ioInputs.tv > 0;
    }

    public double getTX() {
        return ioInputs.tx;
    }

    public void turnOn() { io.setLeds(true); }
    public void turnOff() { io.setLeds(false); }
    
}
