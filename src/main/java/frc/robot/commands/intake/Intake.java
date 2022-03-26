package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.IntakeWheelsIO.IntakeWheelsIOInput;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;

public class Intake extends CommandBase {
    
    private final IntakeWheels intake;
    private final Tower tower;
    private final RotationArms rotationArms;

    public Intake(Tower tower, IntakeWheels intake, RotationArms rotationArms) {

        this.tower = tower;
        this.intake = intake;
        this.rotationArms = rotationArms;

        addRequirements(tower, intake);
    }

    @Override
    public void execute() {

        if (!rotationArms.getKilled() && rotationArms.atGoal() && rotationArms.getGoal() == ClimberConstants.intakePositionRad) {

            /*if ((tower.getDetectedColor().red > 200 && DriverStation.getAlliance() == Alliance.Blue) || (tower.getDetectedColor().blue > 200 && DriverStation.getAlliance() == Alliance.Red)) {
                tower.setIndexWheelsPercent(-BallConstants.towerPower);
                intake.setPercent(-BallConstants.intakePower);
            } else {*/
            if (!tower.getTopSensor()) tower.setConveyorPercent(BallConstants.towerPower);
            else tower.setConveyorPercent(0.0);
    
            tower.setIndexWheelsPercent(BallConstants.towerPower);
            intake.setPercent(BallConstants.intakePower.get());
            //}

        } else {
            intake.setPercent(0);
        }

    }

    @Override
    public void end(boolean isInterrupted) {

        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);
        intake.setPercent(0.0);

    }

}