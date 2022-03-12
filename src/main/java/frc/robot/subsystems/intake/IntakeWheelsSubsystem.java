package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheelsSubsystem extends SubsystemBase {

    private final IntakeWheelsIO io;

    public IntakeWheelsSubsystem(IntakeWheelsIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {
    }

    public void setPercent(double percent) {
        io.setPercent(percent);
    }
    
}
