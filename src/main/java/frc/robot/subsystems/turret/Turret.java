package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    private final TurretIOInputs inputs = new TurretIOInputs();
    private PIDController positionController = new PIDController(TurretConstants.positionKp.get(), 0, TurretConstants.positionKd.get());
    private final TurretIO io;

    public Turret(TurretIO io) {
        this.io = io;

        io.resetEncoderAbsolute();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Turret", inputs);

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
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public double getVelocityRadPerS() {
        return inputs.velocityRadPerS;
    }


}
