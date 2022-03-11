package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;

public class TowerSubsystem extends SubsystemBase {

    private final TowerIO io;
    private final TowerIOInputs ioInputs = new TowerIOInputs();

    public TowerSubsystem(TowerIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Tower", ioInputs);

    }

    void setConveyorPercent(double percent) {
        io.setConveyorPercent(percent);
    }

    void setIndexWheelsPercent(double percent) {
        io.setIndexWheelsPercent(percent);
    }   
    
}
