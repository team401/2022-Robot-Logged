package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.InterpolatingDouble;

public class SegmentedAutoShoot extends CommandBase {

    private final Shooter shooter;
    private final Tower tower;
    private final Vision vision;

    private final Timer timer = new Timer();

    public SegmentedAutoShoot(Shooter shooter, Tower tower, Vision vision) {

        this.shooter = shooter;
        this.tower = tower;
        this.vision = vision;

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        RobotState.AimingParameters params = RobotState.getInstance().getAimingParameters();
        double hoodAngle = ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);

        shooter.setSetpoint(hoodAngle, shotSpeed);

        if (!shooter.atGoal()) {
            timer.reset();
            tower.setConveyorPercent(0.0);
            tower.setIndexWheelsPercent(0.0);
        }

        if (timer.get() > 0.1) {
            tower.setConveyorPercent(1.0);
            tower.setIndexWheelsPercent(1.0);
        }

    }

    @Override
    public boolean isFinished() {
        return timer.get() > 1 || vision.distanceToTargetIn() >= ShooterConstants.maxDistanceToTargetIn;
    }

    @Override
    public void end(boolean isInterrupted) {

        shooter.stopShooter();
        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);

    }
    
}