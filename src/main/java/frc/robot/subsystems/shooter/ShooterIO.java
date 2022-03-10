package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
    public static class ShooterIOInputs implements LoggableInputs {
        public double flywheelSpeedRadPerS;
        public double hoodPositionRad;
        public double[] flywheelCurrent = new double[2];
        public double hoodCurrent;

        @Override
        public void toLog(LogTable table) {
            table.put("FlywheelSpeedRadPerS", flywheelSpeedRadPerS);
            table.put("HoodPositionRad", hoodPositionRad);          
            table.put("FlywheelCurrent", flywheelCurrent);         
            table.put("HoodCurrent", hoodCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            flywheelSpeedRadPerS = table.getDouble("FlywheelSpeedRadPerS", flywheelSpeedRadPerS);
            hoodPositionRad = table.getDouble("HoodPositionRad", hoodPositionRad);          
            flywheelCurrent = table.getDoubleArray("FlywheelCurrent", flywheelCurrent);         
            hoodCurrent = table.getDouble("HoodCurrent", hoodCurrent);          
        }
    }

    void updateInputs(ShooterIOInputs inputs);

    void zeroHoodEncoder();
    void setHoodPositionSetpoint(double angleRad);
    void setHoodVoltage(double volts);
    void setHoodPD(double p, double d);

    void setFlywheelVelocity(double velocityRadPerS, double ffVolts);
    void setFlywheelVoltage(double volts);
    void setFlywheelPD(double p, double d);
}
