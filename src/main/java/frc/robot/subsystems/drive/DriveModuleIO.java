package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveModuleIO {
    public static class DriveIOInputs implements LoggableInputs {
        public double driveVelocityRadPerS;
        public double drivePositionRad;
        public double rotationPositionRad;

        //How the logger gets data
        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocityRadPerS", driveVelocityRadPerS);
            table.put("DrivePositionRad", drivePositionRad);
            table.put("RotationPositionRad", rotationPositionRad);
            
        }

        @Override
        public void fromLog(LogTable table) {
            driveVelocityRadPerS = table.getDouble("DriveVelocityRadPerS", driveVelocityRadPerS);
            driveVelocityRadPerS = table.getDouble("DrivePositionRad", drivePositionRad);
            driveVelocityRadPerS = table.getDouble("RotationPositionRad", rotationPositionRad);
            
        }

    }

    void updateInputs(DriveIOInputs inputs);
    
    void zeroEncoders();
    void setRotationVoltage(double volts);
    void setDriveVoltage(double volts);
    void setDriveVelocity(double velocityRadPerS);
        
}