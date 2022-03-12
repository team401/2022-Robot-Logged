package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeWheelsIO.IntakeWheelsIOInput;
import frc.robot.subsystems.tower.TowerSubsystem;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;

public class Intake extends CommandBase {
    
    private final IntakeSubsystem intake;
    private final TowerSubsystem tower;
    private final TowerIOInputs towerIOInputs;

    public Intake(TowerSubsystem tower, TowerIOInputs towerIOInputs, IntakeSubsystem intake) {

        this.tower = tower;
        this.intake = intake;
        this.towerIOInputs = towerIOInputs;
        
    }


    @Override
    public void execute() {

        if(!towerIOInputs.topSensor) tower.setConveyorPercent(BallConstants.towerPower);
        else tower.setConveyorPercent(0.0);

        tower.setIndexWheelsPercent(BallConstants.towerPower);
        intake.setPercent(BallConstants.intakePower);

    }

    @Override
    public void end(boolean isInterrupted) {

        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);
        intake.setPercent(0.0);

    }


}
