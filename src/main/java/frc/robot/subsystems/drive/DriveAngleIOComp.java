package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;

public class DriveAngleIOComp implements DriveAngleIO {

    private final PigeonIMU imu = new PigeonIMU(CANDevices.pigeonIMU);

    @Override
    public void updateInputs(DriveAngleIOInputs inputs) {
        inputs.headingRad = Units.degreesToRadians(imu.getFusedHeading());
    }

    @Override
    public void resetHeading() {
        imu.setFusedHeading(0);        
    }  
}
