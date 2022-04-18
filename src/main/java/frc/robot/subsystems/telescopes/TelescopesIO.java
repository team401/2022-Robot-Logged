package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TelescopesIO {
    public static class TelescopesIOInputs implements LoggableInputs {
        public double leftPositionRad;
        public double rightPositionRad;
        public double leftVelocityRadPerS;
        public double rightVelocityRadPerS;
        public double leftCurrent;
        public double rightCurrent;

        @Override
        public void toLog(LogTable table) {
            table.put("LeftPositionRad", leftPositionRad);
            table.put("RightPositionRad", rightPositionRad);
            table.put("LeftVelocityRadPerS", leftVelocityRadPerS);
            table.put("RightVelocityRadPerS", rightVelocityRadPerS);
            table.put("LeftCurrent", leftCurrent);
            table.put("RightCurrent", rightCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionRad = table.getDouble("LeftPositionRad", leftPositionRad);
            rightPositionRad = table.getDouble("RightPositionRad", rightPositionRad);
            leftVelocityRadPerS = table.getDouble("LeftVelocityRadPerS", leftVelocityRadPerS);
            rightVelocityRadPerS = table.getDouble("RightVelocityRadPerS", rightVelocityRadPerS);
            leftCurrent = table.getDouble("LeftCurrent", leftCurrent);
            rightCurrent = table.getDouble("RightCurrent", rightCurrent);
        }

    }

    void updateInputs(TelescopesIOInputs inputs);

    void resetLeftEncoder();
    void resetRightEncoder();

    void setLeftVolts(double volts);
    void setRightVolts(double volts);
}
