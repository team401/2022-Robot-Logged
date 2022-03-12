package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shootervision.ShooterVision;

public class PrepareToShoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final ShooterVision vision;

    public PrepareToShoot(ShooterSubsystem shooter, ShooterVision vision) {

        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter, vision);

    }

    @Override
    public void execute() {

        double calculatedFlywheelVelocity = 0;
        double calculatedHoodPosition = 0;

        shooter.setFlywheelVelocity(calculatedFlywheelVelocity, 0);
        shooter.setHoodPositionSetpoint(calculatedHoodPosition);

    }
    
}