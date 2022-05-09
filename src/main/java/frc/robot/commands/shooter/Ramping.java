package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState.AimingParameters;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Interpolation.InterpolatingDouble;

public class Ramping extends CommandBase {

    private final Shooter shooter;

    public Ramping(Shooter shooter) {

        this.shooter = shooter;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        AimingParameters params = RobotState.getInstance().getAimingParameters();
        double hoodAngle = ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);

        shooter.setSetpoint(hoodAngle, shotSpeed);

    }

    public void end(boolean isfinished) {

        shooter.stopShooter();

    }
    
}
