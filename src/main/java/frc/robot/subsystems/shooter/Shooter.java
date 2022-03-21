package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

import frc.robot.commands.shooter.Shoot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputs ioInputs = new ShooterIOInputs();

    private double flywheelGoalRadPerS = 0;
    private double hoodGoalRad = 0;
    private boolean homed = false;
    private boolean hoodEnable = false;
    private boolean flywheelEnable = false;
    private final Timer homeTimer = new Timer();

    private final double flywheelToleranceRadPerS = Units.rotationsPerMinuteToRadiansPerSecond(150);
    private final double hoodToleranceRad = Units.rotationsToRadians(0.25);

    private double rpmOffset = -20;

    public Shooter(ShooterIO io) {
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

        Logger.getInstance().recordOutput("Shooter/RPMOffset", rpmOffset);
        SmartDashboard.putNumber("RPM Offset", rpmOffset);

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
        this.hoodGoalRad = MathUtil.clamp(this.hoodGoalRad, ShooterConstants.hoodMinRad, ShooterConstants.hoodMaxRad);
        this.flywheelGoalRadPerS = flywheelGoalRadPerS;
        if (flywheelGoalRadPerS != 0) this.flywheelGoalRadPerS += Units.rotationsPerMinuteToRadiansPerSecond(rpmOffset);
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

    public double getHoodPositionRad() {
        return ioInputs.hoodPositionRad;
    }

    public boolean atGoal() {
        return Math.abs(ioInputs.flywheelSpeedRadPerS-flywheelGoalRadPerS) < flywheelToleranceRadPerS &&
                Math.abs(ioInputs.hoodPositionRad-hoodGoalRad) < hoodToleranceRad;
    }

    public void killTurret() {
        setFlywheelVolts(0);
        flywheelEnable = false;
    }

    public void killHood() {
        io.setHoodVoltage(0);
        hoodEnable = false;
    }

    public void incrementRPMOffset(int offset) {
        rpmOffset += offset;
    }
    
}
