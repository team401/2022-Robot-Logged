package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.InterpolatingDouble;

public class PrepareToShoot extends CommandBase {

    private final ShooterSubsystem shooter;

    public PrepareToShoot(ShooterSubsystem shooter) {

        this.shooter = shooter;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        RobotState.AimingParameters params = RobotState.getInstance().getAimingParameters();
        double hoodAngle = Constants.ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(Constants.ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);

        shooter.setSetpoint(hoodAngle, shotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}