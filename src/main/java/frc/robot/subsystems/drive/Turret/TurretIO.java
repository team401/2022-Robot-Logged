package frc.robot.subsystems.drive.Turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TurretIO {
    public static class TurretIOInputs implements LoggableInputs {

        public double PositionRad;
        public double VelocityRadPerS;

        @Override
        public void toLog(LogTable table) {
            table.put("PositionRad", PositionRad);
            table.put("VelocityRadPerS", VelocityRadPerS);
        }

        @Override
        public void fromLog(LogTable table) {
            PositionRad = table.getDouble("PositionRad", PositionRad);
            VelocityRadPerS = table.getDouble("VelocityRadPerS", VelocityRadPerS);
        }
    }

    void updateInputs(TurretIOInputs inputs);

    void resetEncoderAbsolute();
    void setSoftLimits();
    void setVoltage(double voltage);
    void setVelocity(double velocityRadPerS, double ffVolts);
}
