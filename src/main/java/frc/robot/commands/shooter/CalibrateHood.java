package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class CalibrateHood extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
  
    private Timer hoodTimer = new Timer();

    public CalibrateHood(ShooterSubsystem shooter) {

        shooterSubsystem = shooter;

        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize() {

        hoodTimer.reset();
        hoodTimer.start();

        shooterSubsystem.setHoodPercent(-0.2);

    }

    @Override
    public void execute() {

        if (Math.abs(shooterSubsystem.getHoodVelocity()) > 0.01) {

            hoodTimer.reset();

        }
        
    }

    @Override
    public boolean isFinished() {

        return hoodTimer.get() >= 0.1;

    }

    @Override
    public void end(boolean isInterrupted) {

        shooterSubsystem.setHoodPercent(0.0);
        shooterSubsystem.zeroHoodEncoder();
        
    }
    
}
