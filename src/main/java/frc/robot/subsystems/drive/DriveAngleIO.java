package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveAngleIO {
    public static class DriveAngleIOInputs implements LoggableInputs {
        public double headingRad;

        @Override
        public void toLog(LogTable table) {
            table.put("HeadingRad", headingRad);
        }

        @Override
        public void fromLog(LogTable table) {
            headingRad = table.getDouble("HeadingRad", headingRad);
        }
    }

    void updateInputs(DriveAngleIOInputs inputs);
    
    void resetHeading();
}
