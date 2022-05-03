package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.Interpolation.InterpolatingDouble;

public class PrepareToShoot extends CommandBase {

    private final Shooter shooter;
    private final DoubleSupplier percent;

    public PrepareToShoot(Shooter shooter, DoubleSupplier supplier) {

        this.shooter = shooter;
        this.percent = supplier;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        double desiredRPM = percent.getAsDouble() > 0.05 ? percent.getAsDouble() * 2000 : 0;
        shooter.setSetpoint(ShooterConstants.hoodMaxRad, Units.rotationsPerMinuteToRadiansPerSecond(desiredRPM));
      
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}