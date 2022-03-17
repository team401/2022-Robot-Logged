package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheels extends SubsystemBase {

    private final IntakeWheelsIO io;

    public IntakeWheels(IntakeWheelsIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {
    }

    public void setPercent(double percent) {
        io.setPercent(percent);
    }
    
}
