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

    public void zeroHoodEncoder() {
        io.zeroHoodEncoder();
    }

    public void setHoodPositionSetpoint(double angleRad) {
        io.setHoodPositionSetpoint(angleRad);
    }

    public void setHoodPercent(double percent) {
        io.setHoodVoltage(percent * 12);
    }

    public void setHoodPD(double p, double d) {
        io.setHoodPD(p, d);
    }

    public void setFlywheelVelocity(double velocityRadPerS, double ffVolts) {
        io.setFlywheelVelocity(velocityRadPerS, ffVolts);
    }

    public void setFlywheelPercent(double percent) {
        io.setFlywheelVoltage(percent * 12);
    }

    public void setFlywheelPD(double p, double d) {
        io.setFlywheelPD(p, d);
    }
    
}
