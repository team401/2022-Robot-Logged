package frc.robot.subsystems.rotationArms;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface RotationArmsIO {
    public static class RotationArmIOInputs implements LoggableInputs {
        public double leftPositionRad;
        public double rightPositionRad;

        @Override
        public void toLog(LogTable table) {
            table.put("RotationArmLeftPositionRad", leftPositionRad);
            table.put("RotationArmRightPositionRad", rightPositionRad);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPositionRad = table.getDouble("RotationArmLeftPositionRad", leftPositionRad);
            rightPositionRad = table.getDouble("RotationArmRightPositionRad", rightPositionRad);
        }

    }

    void updateInputs(RotationArmIOInputs inputs);

    void resetControllers();

    void setLeftPercent(double percent);
    void setRightPercent(double percent);

    void setLeftDesiredPositionRad(double desiredRadians);
    void setRightDesiredPositionRad(double desiredRadians);

    void kill();

    void setPD(double p, double d);
        
}