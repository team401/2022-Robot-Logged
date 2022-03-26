package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

    private final Timer lastSensorUpdateTimer = new Timer();
    private int lastSensorRedValue = 0;

    public Intake(Tower tower, IntakeWheels intake, RotationArms rotationArms) {

        this.tower = tower;
        this.intake = intake;
        this.rotationArms = rotationArms;

        addRequirements(tower, intake);
    }

    @Override
    public void initialize() {
        lastSensorUpdateTimer.reset();
        lastSensorUpdateTimer.start();
    }

    @Override
    public void execute() {

        if (lastSensorRedValue != tower.getDetectedColor().red)
            lastSensorUpdateTimer.reset();
        lastSensorRedValue = tower.getDetectedColor().red;

        if (rotationArms.atGoal() && rotationArms.getGoal() == ClimberConstants.intakePositionRad) {

            boolean invalidBall = (tower.getDetectedColor().red > 60 && DriverStation.getAlliance() == Alliance.Blue) ||  
                                    (tower.getDetectedColor().red < 60 && DriverStation.getAlliance() == Alliance.Red);
            if (invalidBall && lastSensorUpdateTimer.get() < 0.1) {
                tower.setIndexWheelsPercent(-BallConstants.towerPower);
                intake.setPercent(-BallConstants.intakePower.get());
            } else {
                if (!tower.getTopSensor()) tower.setConveyorPercent(BallConstants.towerPower);
                else tower.setConveyorPercent(0.0);
        
                tower.setIndexWheelsPercent(BallConstants.towerPower);
                intake.setPercent(BallConstants.intakePower.get());
            }

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