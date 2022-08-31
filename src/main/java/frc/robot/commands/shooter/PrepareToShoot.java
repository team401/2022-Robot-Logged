package frc.robot.commands.shooter;

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
    private final Tower tower;

    public PrepareToShoot(Shooter shooter, Tower tower) {

        this.shooter = shooter;
        this.tower = tower;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        RobotState.AimingParameters params = RobotState.getInstance().getAimingParameters();
        double hoodAngle = Constants.ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(Constants.ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);

        if (RobotState.getInstance().hasCorrectBall())
            shooter.setSetpoint(hoodAngle, shotSpeed);
        else
            shooter.setSetpoint(hoodAngle, Units.rotationsPerMinuteToRadiansPerSecond(Constants.ShooterConstants.intentionalMissRPM));
        
        //shooter.setSetpoint(SmartDashboard.getNumber("Hood Desired", 0.27), Units.rotationsPerMinuteToRadiansPerSecond(SmartDashboard.getNumber("Shooter Desired", 0)));
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}