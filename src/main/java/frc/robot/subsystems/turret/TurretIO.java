package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TurretIO {
    public static class TurretIOInputs implements LoggableInputs {
        public double absolutePositionRad;
        public double positionRad;
        public double velocityRadPerS;
        public double current;

        @Override
        public void toLog(LogTable table) {
            table.put("AbsolutePositionRad", absolutePositionRad);
            table.put("PositionRad", positionRad);
            table.put("VelocityRadPerS", velocityRadPerS);
            table.put("Current", current);
        }

        @Override
        public void fromLog(LogTable table) {
            absolutePositionRad = table.getDouble("AbsolutePositionRad", absolutePositionRad);
            positionRad = table.getDouble("PositionRad", positionRad);
            velocityRadPerS = table.getDouble("VelocityRadPerS", velocityRadPerS);
            current = table.getDouble("Current", current);
        }
    }

    void updateInputs(TurretIOInputs inputs);

    void resetEncoder();

    void setVoltage(double voltage);
    void setVelocitySetpoint(double velocityRadPerS, double ffVolts);

    void setVelocityPD(double p, double d);

    double getCurrent();

    public void setNeutralMode(NeutralMode mode);
    
}
