package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TelescopesIO {
    public static class TelescopesIOInputs implements LoggableInputs {

        public double leftPositionM;
        public double rightPositionM;
        public double leftCurrent;
        public double rightCurrent;

        @Override
        public void toLog(LogTable table) {
            table.put("LeftPositionM", leftPositionM);
            table.put("RightPositionM", rightPositionM);
            table.put("LeftCurrent", leftCurrent);
            table.put("RightCurrent", rightCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionM = table.getDouble("LeftPositionM", leftPositionM);
            rightPositionM = table.getDouble("RightPositionM", rightPositionM);
            leftCurrent = table.getDouble("LeftCurrent", leftCurrent);
            rightCurrent = table.getDouble("RightCurrent", rightCurrent);
        }

    }

    void updateInputs(TelescopesIOInputs inputs);

    void setLeftVolts(double volts);
    void setRightVolts(double volts);

    double getLeftCurrentDraw();
    double getRightCurrentDraw();

    boolean isLeftLidarOnline();
    boolean isRightLidarOnline();
}
