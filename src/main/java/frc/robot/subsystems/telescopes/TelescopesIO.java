package frc.robot.subsystems.telescopes;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TelescopesIO {
    public static class TelescopesIOInputs implements LoggableInputs {

        public double leftLidarPositionM;
        public double rightLidarPositionM;
        public double leftEncoderPositionM;
        public double rightEncoderPositionM;
        public double leftVelocityMPerS;
        public double rightVelocityMPerS;
        public double leftCurrent;
        public double rightCurrent;

        @Override
        public void toLog(LogTable table) {
            table.put("LeftLidarPositionM", leftLidarPositionM);
            table.put("RightLidarPositionM", leftLidarPositionM);
            table.put("LeftEncoderPositionM", leftEncoderPositionM);
            table.put("RightEncoderPositionM", leftEncoderPositionM);
            table.put("LeftVelocityMPerS", leftVelocityMPerS);
            table.put("RightVelocityMPerS", rightVelocityMPerS);
            table.put("LeftCurrent", leftCurrent);
            table.put("RightCurrent", rightCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            leftLidarPositionM = table.getDouble("LeftLidarPositionM", leftLidarPositionM);
            rightLidarPositionM = table.getDouble("RightLidarPositionM", rightLidarPositionM);
            leftEncoderPositionM = table.getDouble("LeftEncoderPositionM", leftEncoderPositionM);
            rightEncoderPositionM = table.getDouble("RightEncoderPositionM", rightEncoderPositionM);
            leftVelocityMPerS = table.getDouble("LeftVelocityMPerS", leftVelocityMPerS);
            rightVelocityMPerS = table.getDouble("RightVelocityMPerS", rightVelocityMPerS);
            leftCurrent = table.getDouble("LeftCurrent", leftCurrent);
            rightCurrent = table.getDouble("RightCurrent", rightCurrent);
        }

    }

    void updateInputs(TelescopesIOInputs inputs);

    void setLeftEncoder(double positionM);
    void setRightEncoder(double positionM);

    void setLeftVolts(double volts);
    void setRightVolts(double volts);

    double getLeftCurrentDraw();
    double getRightCurrentDraw();

    boolean isLeftLidarValid();
    boolean isRightLidarValid();
}
