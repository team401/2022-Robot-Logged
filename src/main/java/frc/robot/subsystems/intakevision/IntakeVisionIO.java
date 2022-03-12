package frc.robot.subsystems.intakevision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeVisionIO {
    public static class IntakeVisionIOInputs implements LoggableInputs {

        public double tx;
        public double ta;
        public double tv;

        @Override
        public void toLog(LogTable table) {
            table.put("tx", tx);
            table.put("ta", ta);
            table.put("tv", tv);
        }

        @Override
        public void fromLog(LogTable table) {
            tx = table.getDouble("tx", tx);
            ta = table.getDouble("ta", ta);
            tv = table.getDouble("tv", tv);
        }
    }

    void updateInputs(IntakeVisionIOInputs inputs);
    void setLeds(boolean enabled);
    
}