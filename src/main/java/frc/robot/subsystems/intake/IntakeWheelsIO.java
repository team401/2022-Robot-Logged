package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeWheelsIO {
    public static class IntakeWheelsIOInput implements LoggableInputs {
        public double current;
        
        //How the logger gets data
        @Override
        public void toLog(LogTable table) {
            table.put("Current", current);
        }

        @Override
        public void fromLog(LogTable table) {
            current = table.getDouble("Current", current);
        }

    }

    void updateInputs(IntakeWheelsIOInput inputs);
    void setPercent(double percent);
    
}