package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;

public class DriveAngleIOComp implements DriveAngleIO {

    private final Pigeon2 pigeon = new Pigeon2(CANDevices.pigeonIMU);

    private double degOffset = 0;

    @Override
    public void updateInputs(DriveAngleIOInputs inputs) {
        inputs.headingRad = Units.degreesToRadians(pigeon.getYaw() - degOffset);
    }

    @Override
    public void resetHeading() {
        degOffset = pigeon.getYaw();
    }  
}