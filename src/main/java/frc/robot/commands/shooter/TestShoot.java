package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;

public class TestShoot extends CommandBase {

    private final Tower tower;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    private boolean hasShot = false;
    private boolean prevAtGoal = false;


    /*private final Timer notAtGoalTimer = new Timer();
    private final Timer reverseTimer = new Timer();
    private boolean notAtGoalTimerStarted = false;*/

    public TestShoot(Tower tower, Shooter shooter) {
        this.shooter = shooter;
        this.tower = tower;

        addRequirements(tower);

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        //notAtGoalTimer.reset();
        //notAtGoalTimer.stop();

    }

    @Override
    public void execute() {

        if (hasShot && prevAtGoal && !shooter.atGoal()) {

        }

        if (!shooter.atGoal()) {
            timer.reset();
            tower.setConveyorPercent(0.0);
            tower.setIndexWheelsPercent(0.0);
        }

        if (timer.get() > 0.1) {
            tower.setConveyorPercent(1.0);
            tower.setIndexWheelsPercent(1.0);
            hasShot = true;
        }

        prevAtGoal = shooter.atGoal();

    }

    @Override
    public void end(boolean isInterrupted) {
        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);
    }
    
}
