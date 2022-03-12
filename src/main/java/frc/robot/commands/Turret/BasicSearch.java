package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;

public class BasicSearch extends CommandBase{

    private final limelightSubsystem limelight;
    private final TurretSubsystem turret;

    public BasicSearch(limelightSubsystem limelight, TurretSubsystem turret) {

        this.limelight = limelight;
        this.turret = turret;

        // Use the limelight and turret subsystems
        addRequirements(limelight, turret);

    }

    @Override
    public 

    
}
