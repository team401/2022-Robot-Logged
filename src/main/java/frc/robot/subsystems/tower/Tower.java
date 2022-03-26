package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;
import frc.robot.util.PicoColorSensor.RawColor;

public class Tower extends SubsystemBase {

    private final TowerIO io;
    private final TowerIOInputs ioInputs = new TowerIOInputs();

    public Tower(TowerIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Tower", ioInputs);

    }

    public void setConveyorPercent(double percent) {
        io.setConveyorPercent(percent);
    }

    public void setIndexWheelsPercent(double percent) {
        io.setIndexWheelsPercent(percent);
    }   

    public boolean getTopSensor() {
        return ioInputs.topSensor;
    }

    public RawColor getDetectedColor() {
        return ioInputs.detectedColor;
    }
    
}
