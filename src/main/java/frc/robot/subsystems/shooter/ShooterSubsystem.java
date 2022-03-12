package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterConstants;

import frc.robot.commands.shooter.Shoot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputs ioInputs = new ShooterIOInputs();

    private double flywheelGoalRadPerS = 0;
    private double hoodGoalRad = 0;
    private boolean homed = false;
    private boolean hoodEnable = false;
    private boolean flywheelEnable = false;
    private final Timer homeTimer = new Timer();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Shooter", ioInputs);

        if (ShooterConstants.hoodKp.hasChanged() || ShooterConstants.hoodKd.hasChanged()) {
            io.setHoodPD(ShooterConstants.hoodKp.get(), ShooterConstants.hoodKd.get());
        }

        if (ShooterConstants.flywheelKp.hasChanged() || ShooterConstants.flywheelKd.hasChanged()) {
            io.setFlywheelPD(ShooterConstants.flywheelKp.get(), ShooterConstants.flywheelKd.get());
        }

        Logger.getInstance().recordOutput("Shooter/HoodAngleDeg", Units.radiansToDegrees(ioInputs.hoodPositionRad));
        Logger.getInstance().recordOutput("Shooter/HoodSetpointDeg", Units.radiansToDegrees(hoodGoalRad));
        Logger.getInstance().recordOutput("Shooter/FlywheelRPM", Units.radiansPerSecondToRotationsPerMinute(ioInputs.flywheelSpeedRadPerS));
        Logger.getInstance().recordOutput("Shooter/FlywheelSetpointRPM", Units.radiansPerSecondToRotationsPerMinute(flywheelGoalRadPerS));

        if (!homed) {
            if (DriverStation.isEnabled()) {
                if (Math.abs(ioInputs.hoodVelocity) < ShooterConstants.hoodHomingThresholdRadPerS) {
                    homeTimer.start();
                } else {
                    homeTimer.stop();
                    homeTimer.reset();
                }

                if (homeTimer.hasElapsed(ShooterConstants.hoodHomingTimeS)) {
                    homed = true;
                    hoodEnable = false;
                    io.zeroHoodEncoder();
                    io.setHoodVoltage(0);
                } else {
                    io.setHoodVoltage(ShooterConstants.hoodHomingVolts);
                }
            }
        } else {
            if (hoodEnable) {
                io.setHoodPositionSetpoint(hoodGoalRad);
            } else {
                io.setHoodVoltage(0);
            }

            if (flywheelEnable) {
                io.setFlywheelVelocity(flywheelGoalRadPerS, ShooterConstants.flywheelModel.calculate(flywheelGoalRadPerS));
            } else {
               io.setFlywheelVoltage(0);
            }
        }


    }

    public void zeroHoodEncoder() {
        io.zeroHoodEncoder();
    }
    
    public double getHoodVelocity() {
        return ioInputs.hoodVelocity;

    }

    public void setSetpoint(double hoodAngleRad, double flywheelGoalRadPerS) {
        hoodEnable = true;
        flywheelEnable = true;
        this.hoodGoalRad = hoodAngleRad;
        this.flywheelGoalRadPerS = flywheelGoalRadPerS;
    }

    public void stopShooter() {
        hoodEnable = false;
        flywheelEnable = false;
    }

    public void setFlywheelVolts(double volts) {
        io.setFlywheelVoltage(volts);
    }

    public double getFlywheelVelocityRadPerS() {
        return ioInputs.flywheelSpeedRadPerS;
    }
    
}
