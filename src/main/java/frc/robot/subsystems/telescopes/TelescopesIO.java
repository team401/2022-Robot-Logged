package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TelescopesIO {
    public static class TelescopesIOInputs implements LoggableInputs {
        public double leftPositionRad;
        public double rightPositionRad;

        @Override
        public void toLog(LogTable table) {
            table.put("Telescope Left Position Rad", leftPositionRad);
            table.put("Telescope Right Position Rad", rightPositionRad);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionRad = table.getDouble("Telescope Left Position Rad", leftPositionRad);
            rightPositionRad = table.getDouble("Telescope Right Position Rad", rightPositionRad);
        }

    }

    void updateInputs(TelescopesIOInputs inputs);

    void resetLeftEncoder();
    void resetRightEncoder();

    void resetControllers();

    void setLeftPercent(double volts);
    void setRightPercent(double volts);

    void setLeftDesiredPositionRad(double desiredPosition);
    void setRightDesiredPositionRad(double desiredPosition);

    void setPD(double p, double d);
}
