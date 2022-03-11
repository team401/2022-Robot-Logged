package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TelescopesIO {
    public static class TelescopesIOInputs implements LoggableInputs {
        public double leftPositionIn;
        public double rightPositionIn;
        public double leftCurrent;
        public double rightCurrent;

        @Override
        public void toLog(LogTable table) {
            table.put("LeftPositionIn", leftPositionIn);
            table.put("RightPositionIn", rightPositionIn);
            table.put("LeftCurrent", leftCurrent);
            table.put("RightCurrent", rightCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionIn = table.getDouble("LeftPositionIn", leftPositionIn);
            rightPositionIn = table.getDouble("RightPositionIn", rightPositionIn);
            leftCurrent = table.getDouble("LeftCurrent", leftCurrent);
            rightCurrent = table.getDouble("RightCurrent", rightCurrent);
        }

    }

    void updateInputs(TelescopesIOInputs inputs);

    void resetEncoders();

    void setLeftVolts(double volts);
    void setRightVolts(double volts);
}
