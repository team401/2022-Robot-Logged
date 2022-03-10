package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TelescopesIO {
    public static class TelescopesIOInputs implements LoggableInputs {
        public double leftPositionMeters;
        public double rightPositionMeters;

        @Override
        public void toLog(LogTable table) {
            table.put("TelescopeLeftPositionMeters", leftPositionMeters);
            table.put("TelescopeRightPositionMeters", rightPositionMeters);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionMeters = table.getDouble("TelescopeLeftPositionMeters", leftPositionMeters);
            rightPositionMeters = table.getDouble("TelescopeRightPositionMeters", rightPositionMeters);
        }

    }

    void updateInputs(TelescopesIOInputs inputs);

    void resetLeftEncoder();
    void resetRightEncoder();

    void resetControllers();

    void setLeftVoltage(double volts);
    void setRightVoltage(double volts);

    void setLeftDesiredPositionMeters(double desiredPosition);
    void setRightDesiredPositionMeters(double desiredPosition);

    void setPD(double p, double d);
}
