package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.Interpolation.InterpolatingDouble;

public class PrepareToShoot extends CommandBase {

    private final Shooter shooter;
    private final Tower tower;

    public PrepareToShoot(Shooter shooter, Tower tower) {

        this.shooter = shooter;
        this.tower = tower;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        /*if ((DriverStation.getAlliance() == Alliance.Blue && tower.getTopBall() == BallType.Red) ||
            (DriverStation.getAlliance() == Alliance.Red && tower.getTopBall() == BallType.Blue)) {
            shooter.setSetpoint(0.35, Units.rotationsPerMinuteToRadiansPerSecond(1000));
        }
        else {*/
            RobotState.AimingParameters params = RobotState.getInstance().getAimingParameters();
            double hoodAngle = Constants.ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
            double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(Constants.ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);

            shooter.setSetpoint(hoodAngle, shotSpeed);
        //}

        //shooter.setSetpoint(ShooterConstants.hoodDesired.get(), Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.flywheelDesired.get()));
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}