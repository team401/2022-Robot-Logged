package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;

public class Shoot extends CommandBase {

    private final Tower tower;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    

    public Shoot(Tower tower, Shooter shooter) {

        this.shooter = shooter;
        this.tower = tower;

        addRequirements(tower);

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

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
    public void end(boolean isInterrupted) {
        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);
    }
    
}
