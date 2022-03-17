package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallConstants;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.IntakeWheelsIO.IntakeWheelsIOInput;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;

public class Intake extends CommandBase {
    
    private final IntakeWheels intake;
    private final Tower tower;

    public Intake(Tower tower, IntakeWheels intake) {

        this.tower = tower;
        this.intake = intake;
        
    }


    @Override
    public void execute() {

        if(!tower.getTopSensor()) tower.setConveyorPercent(BallConstants.towerPower);
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
