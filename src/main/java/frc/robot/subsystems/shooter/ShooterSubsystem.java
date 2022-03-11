package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputs ioInputs = new ShooterIOInputs();

    public ShooterSubsystem(ShooterIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Shooter", ioInputs);

    }

    void zeroHoodEncoder() {
        io.zeroHoodEncoder();
    }

    void setHoodPositionSetpoint(double angleRad) {
        io.setHoodPositionSetpoint(angleRad);
    }

    void setHoodPercent(double percent) {
        io.setHoodVoltage(percent * 12);
    }

    void setHoodPD(double p, double d) {
        io.setHoodPD(p, d);
    }

    void setFlywheelVelocity(double velocityRadPerS, double ffVolts) {
        io.setFlywheelVelocity(velocityRadPerS, ffVolts);
    }

    void setFlywheelPercent(double percent) {
        io.setFlywheelVoltage(percent * 12);
    }

    void setFlywheelPD(double p, double d) {
        io.setFlywheelPD(p, d);
    }
    
}
