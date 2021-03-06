package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    private final TurretIOInputs inputs = new TurretIOInputs();
    private PIDController positionController = new PIDController(TurretConstants.positionKp.get(), 0, TurretConstants.positionKd.get());
    private final TurretIO io;
    private Rotation2d goalPosition = new Rotation2d();
    private double velocityGoal = 0;

    private double encoderOffset = 0;
    private int setupCycleCount = 0;

    private boolean killed = false;
    private double lastUpdateValue = 0;
    private long lastUpdateTimeMS = System.currentTimeMillis();
    
    private boolean zeroOverride = false;

    private final Timer overdrawTimer = new Timer();

    public Turret(TurretIO io) {
        this.io = io;
        
        io.resetEncoder();

        positionController.setTolerance(Units.degreesToRadians(3));

        lastUpdateTimeMS = System.currentTimeMillis();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Turret", inputs);
        
        if (setupCycleCount == TurretConstants.setupCycleCount) {
            io.resetEncoder();
            encoderOffset = MathUtil.angleModulus(inputs.absolutePositionRad);
            setupCycleCount++;
            lastUpdateValue = inputs.positionRad + encoderOffset;
        }
        else {
            setupCycleCount++;
        }
        
        if (!DriverStation.isEnabled()) {
            io.setNeutralMode(NeutralMode.Coast);
        }
        else {
            io.setNeutralMode(NeutralMode.Brake);
        }
        
        // Update gains if they have changed
        if (TurretConstants.positionKp.hasChanged() || TurretConstants.positionKd.hasChanged()) {
            positionController.setP(TurretConstants.positionKp.get());
            positionController.setD(TurretConstants.positionKd.get());
        }
        
        if (TurretConstants.velocityKp.hasChanged() || TurretConstants.velocityKd.hasChanged()) {
            io.setVelocityPD(TurretConstants.velocityKp.get(), TurretConstants.velocityKd.get());
        }
        
        double turretRotation = inputs.positionRad + encoderOffset;
        Logger.getInstance().recordOutput("Turret/Beyond Boundaries", Math.abs(turretRotation) > TurretConstants.turretLimitUpper);
        Logger.getInstance().recordOutput("Turret/RotationDeg", Units.radiansToDegrees(turretRotation));
        Logger.getInstance().recordOutput("Turret/SetpointDeg", goalPosition.getDegrees());
        Logger.getInstance().recordOutput("Turret/VelocityFFDegPerSec", Units.radiansToDegrees(velocityGoal));
        Logger.getInstance().recordOutput("Turret/Killed", killed);
        Logger.getInstance().recordOutput("Turret/Current Draw", io.getCurrent());
        SmartDashboard.putNumber("Turret Rotation", turretRotation);

        if (inputs.absolutePositionRad != lastUpdateValue) {
            lastUpdateTimeMS = System.currentTimeMillis();
        }
        lastUpdateValue = inputs.absolutePositionRad;

        if (System.currentTimeMillis()-lastUpdateTimeMS > 500 && setupCycleCount > TurretConstants.setupCycleCount && DriverStation.isEnabled())
            killed = true;
        
        if (Math.abs(turretRotation) > TurretConstants.turretLimitUpper + Math.PI/2 && setupCycleCount > TurretConstants.setupCycleCount && DriverStation.isEnabled())
            killed = true;
        
        //PID control - equivalent of our old setdesiredpositionclosedloop methods continuously
        double output = positionController.calculate(turretRotation, zeroOverride ? 0 : goalPosition.getRadians());
        // Only add feed velocity if we are not at our hard stops
        if (goalPosition.getRadians() > TurretConstants.turretLimitLower && goalPosition.getRadians() < TurretConstants.turretLimitUpper) {
            output += TurretConstants.turretModel.calculate(velocityGoal);
        }
        Logger.getInstance().recordOutput("Turret/Output", output);
        if (setupCycleCount > TurretConstants.setupCycleCount && !killed)
            io.setVoltage(output);
        else
            io.setVoltage(0);

        RobotState.getInstance().recordTurretObservations(new Rotation2d(turretRotation), inputs.velocityRadPerS);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPositionGoal(Rotation2d goal, double velocity) {

        velocityGoal = velocity;
        double goalWrapped = MathUtil.angleModulus(goal.getRadians());
        
        //clamps max values to be within -90 and 90 deg
        goalWrapped = MathUtil.clamp(goalWrapped, TurretConstants.turretLimitLower, TurretConstants.turretLimitUpper);
        this.goalPosition = new Rotation2d(goalWrapped);
    }

    public void setPositionGoal(Rotation2d goal) {

        setPositionGoal(goal, 0);

    }

    public double getVelocityRadPerS() {
        return inputs.velocityRadPerS;
    }

    public boolean atGoal() {

        return positionController.atSetpoint();

    }

    public void setZeroOverride(boolean zero) {
        zeroOverride = zero;
    }

    public void kill() {
        killed = true;
    }

    public void unkill() {
        killed = false;
    }

}
