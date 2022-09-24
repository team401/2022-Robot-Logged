package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveModuleIO {
    public static class DriveModuleIOInputs implements LoggableInputs {
        public double driveVelocityRadPerS;
        public double drivePositionRad;
        public double rotationPositionRad;

        //How the logger gets data
        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocityRadPerS", driveVelocityRadPerS);
            table.put("DrivePositionRad", drivePositionRad);
            table.put("RotationPositionRad", rotationPositionRad%(Math.PI * 2) - Math.PI);
            
        }

        @Override
        public void fromLog(LogTable table) {
            driveVelocityRadPerS = table.getDouble("DriveVelocityRadPerS", driveVelocityRadPerS);
            drivePositionRad = table.getDouble("DrivePositionRad", drivePositionRad);
            rotationPositionRad = table.getDouble("RotationPositionRad", rotationPositionRad);
        }

    }

    void updateInputs(DriveModuleIOInputs inputs);
    
    void zeroEncoders();
    void setRotationVoltage(double volts);
    void setDriveVoltage(double volts);
    void setDriveVelocity(double velocityRadPerS, double ffVolts);

    void setDrivePD(double p, double d);
        
}