package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Interpolation.InterpolatingDouble;

public class AutoShoot extends CommandBase {

    private final Shooter shooter;
    private final Tower tower;
    private final Vision vision;

    private final Timer shotTimer = new Timer();
    private boolean timerStarted = false;

    public AutoShoot(Shooter shooter, Tower tower, Vision vision) {
        this.shooter = shooter;
        this.tower = tower;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        shotTimer.reset();
        shotTimer.stop();
    }

    @Override
    public void execute() {

        RobotState.AimingParameters params = RobotState.getInstance().getAimingParameters();
        double hoodAngle = ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);

        shooter.setSetpoint(hoodAngle, shotSpeed);

        if (vision.distanceToTargetIn() < ShooterConstants.maxDistanceToTargetIn && shooter.atGoal()) {

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
