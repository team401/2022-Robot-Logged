package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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
    private double goalPosition = 0;
    private double velocityGoal = 0;

    public Turret(TurretIO io) {
        this.io = io;
        

        io.resetEncoderAbsolute();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Turret", inputs);

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

        Rotation2d turretRotation = new Rotation2d(MathUtil.angleModulus(inputs.positionRad));
        Logger.getInstance().recordOutput("Turret/RotationDeg", turretRotation.getDegrees());
        Logger.getInstance().recordOutput("Turret/SetpointDeg", Units.radiansToDegrees(goalPosition));
        Logger.getInstance().recordOutput("Turret/VelocityFFDegPerSec", Units.radiansToDegrees(velocityGoal));

        //PID control - equivalent of our old setdesiredpositionclosedloop methods continuously
        double output = positionController.calculate(inputs.positionRad, goalPosition);
        // Only add feed velocity if we are not at our hard stops
        if (goalPosition > TurretConstants.turretLimitLower && goalPosition < TurretConstants.turretLimitUpper) {
            output += TurretConstants.turretModel.calculate(velocityGoal);
        }
        io.setVoltage(output);

        RobotState.getInstance().recordTurretObservations(turretRotation, inputs.velocityRadPerS);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPositionGoal(Rotation2d goal, double velocity) {

        velocityGoal = velocity;
        double goalWrapped = MathUtil.angleModulus(goal.getRadians());
        
        //clamps max values to be within -90 and 90 deg
        goalWrapped = MathUtil.clamp(goalWrapped, TurretConstants.turretLimitLower, TurretConstants.turretLimitUpper);
        this.goalPosition = goalWrapped;
    }

    public void setPositionGoal(Rotation2d goal) {

        setPositionGoal(goal, 0);

    }

    /**
     * Sets an absolute position target for the turret.  This can be used to force the turret to, for example,
     * move to positive 180 degrees (something that would not be possible with setPositionGoal as 180 degrees is the
     * wraparound point of Rotation2d objects).
     *
     * @param positionRad The absolute position target for the turret, in radians.
     */
    public void setAbsolutePositionGoal(double positionRad) {
        // Clamp to absolute maximum range of motion
        // Negative contraint was -180 instead of -90
        positionRad = MathUtil.clamp(positionRad, -Math.PI / 2.0, Math.PI / 2.0); 
        this.goalPosition = positionRad;
    }

    public double getVelocityRadPerS() {
        return inputs.velocityRadPerS;
    }


}
