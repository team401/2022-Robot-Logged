package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
    public static class ShooterIOInputs implements LoggableInputs {

        public double shooterRPM;
        public double hoodPosition;
        public double hoodVelocity;

        @Override
        public void toLog(LogTable table) {
            table.put("Shooter Speed (RPM)", shooterRPM);
            table.put("Hood Position (rotations)", hoodPosition);          
            table.put("Hood Velocity (rad/s)", hoodVelocity);            
  
        }

        @Override
        public void fromLog(LogTable table) {
            shooterRPM = table.getDouble("Shooter Speed (RPM)", shooterRPM);
            hoodPosition = table.getDouble("Hood Position (rotations)", hoodPosition); 
            hoodVelocity = table.getDouble("Hood Velocity (rad/s)", hoodVelocity);            
        }

    }

    void updateInputs(ShooterIOInputs inputs);

    void zeroHoodEncoder();
    void hoodSetPosition(double position);
    void hoodSetPercent(double percent);
    void setHoodSoftLimits(float forward, float reverse);
    void setShooterPercent(double percent);
    void setShooterSpeed(double desiredRPM);

    void setShooterPD(double p, double d);
    void setTrapezoidalConstraints(double velocity, double acceleration);


}
