package frc.robot.subsystems.rotationarms;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface RotationArmsIO {
    public static class RotationArmsIOInputs implements LoggableInputs {
        public double leftPositionRad;
        public double rightPositionRad;
        public double leftCurrent;
        public double rightCurrent;

        public double leftVelocityRadPerS;
        public double rightVelocityRadPerS;

        @Override
        public void toLog(LogTable table) {
            table.put("LeftPositionRad", leftPositionRad);
            table.put("RightPositionRad", rightPositionRad);
            table.put("LeftCurrent", leftCurrent);
            table.put("RightCurrent", rightCurrent);
            table.put("RightVelocityRadPerS", rightVelocityRadPerS);
            table.put("LeftVelocityRadPerS", leftVelocityRadPerS);

        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionRad = table.getDouble("LeftPositionRad", leftPositionRad);
            rightPositionRad = table.getDouble("RightPositionRad", rightPositionRad);
            leftCurrent = table.getDouble("LeftCurrent", leftCurrent);
            rightCurrent = table.getDouble("RightCurrent", rightCurrent);
            leftVelocityRadPerS = table.getDouble("LeftVelocityRadPerS", leftVelocityRadPerS);
            rightVelocityRadPerS = table.getDouble("RightVelocityRadPerS", rightVelocityRadPerS);
        }
    }

    void updateInputs(RotationArmsIOInputs inputs);

    void resetEncoder();

    void setLeftVolts(double volts);
    void setRightVolts(double volts);
}