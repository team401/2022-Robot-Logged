package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TowerIO {
    public static class TowerIOInputs implements LoggableInputs {
        public boolean topState;
        public boolean bottomState;

        //How the logger gets data
        @Override
        public void toLog(LogTable table) {
            table.put("Top State", topState);
            table.put("Bottom State", bottomState);
        }

        @Override
        public void fromLog(LogTable table) {
            topState = table.getBoolean("Top State", topState);
            bottomState = table.getBoolean("Bottom State", bottomState);
        }

    }

    void updateInputs(TowerIOInputs inputs);
    void setConveyorPercent(double percent);
    void setIndexWheelsPercent(double percent);    
    
}
