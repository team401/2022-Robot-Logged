package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TowerIO {
    public static class TowerIOInputs implements LoggableInputs {
        public boolean topSensor;
        public boolean bottomSensor;

        public double conveyorCurrent;
        public double indexCurrent;

        //How the logger gets data
        @Override
        public void toLog(LogTable table) {
            table.put("TopSensor", topSensor);
            table.put("BottomSensor", bottomSensor);
            table.put("ConveyorCurrent", conveyorCurrent);
            table.put("IndexCurrent", indexCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            topSensor = table.getBoolean("TopSensor", topSensor);
            bottomSensor = table.getBoolean("BottomSensor", bottomSensor);
            conveyorCurrent = table.getDouble("ConveyorCurrent", conveyorCurrent);
            indexCurrent = table.getDouble("IndexCurrent", indexCurrent);
        }

    }

    void updateInputs(TowerIOInputs inputs);
    void setConveyorPercent(double percent);
    void setIndexWheelsPercent(double percent);    
    
}
