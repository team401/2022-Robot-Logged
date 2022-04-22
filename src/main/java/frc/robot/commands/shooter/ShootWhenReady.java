package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Interpolation.InterpolatingDouble;

public class ShootWhenReady extends CommandBase {

    private final Shooter shooter;
    private final Tower tower;
    private final RobotState robotState;

    private final Timer shooterRampTimer = new Timer();
    private final Timer isStoppedTimer = new Timer();

    public ShootWhenReady(Shooter shooter, Tower tower) {

        this.shooter = shooter;
        this.tower = tower;
        robotState = RobotState.getInstance();
        
        addRequirements(shooter, tower);

    }

    @Override
    public void initialize() {

        shooterRampTimer.reset();
        shooterRampTimer.start();

        isStoppedTimer.reset();
        isStoppedTimer.start();

    }

    @Override
    public void execute() {

        // Stopped
        ChassisSpeeds speeds = robotState.getVehicleVelocity();
        double velocity = Math.sqrt(speeds.vxMetersPerSecond*speeds.vxMetersPerSecond+speeds.vyMetersPerSecond*speeds.vyMetersPerSecond);
        if (velocity > 0.1) {
            isStoppedTimer.reset();
        }
        Logger.getInstance().recordOutput("Drive/Velocity", velocity);

        // Prepare to shoot
        RobotState.AimingParameters params = robotState.getAimingParameters();
        double hoodAngle = ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);
        shooter.setSetpoint(hoodAngle, shotSpeed);

        // Shooting
        if (!shooter.atGoal()) {
            shooterRampTimer.reset();
            tower.setConveyorPercent(0.0);
            tower.setIndexWheelsPercent(0.0);
        }

        if (shooterRampTimer.get() > 0.1 && isStoppedTimer.get() > 0.1 && Vision.getTX() < 5) {
            tower.setConveyorPercent(1.0);
            tower.setIndexWheelsPercent(1.0);
        }
        else {
            tower.setConveyorPercent(0.0);
            tower.setIndexWheelsPercent(0.0);
        }

    }

    @Override
    public void end(boolean isInterrupted) {
        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);
        shooter.stopShooter();
    }

    
}
