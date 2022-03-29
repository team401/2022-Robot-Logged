package frc.robot.commands.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private int lastSensorRed = 0;

    private final Timer reverseTimer = new Timer();

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

        reverseTimer.reset();
        reverseTimer.stop();
    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("Reverse Timer", reverseTimer.get());

        SmartDashboard.putString("Alliance", (DriverStation.getAlliance() == Alliance.Blue ? "Blue" : "Red"));

        if (tower.getDetectedColor().red != lastSensorRed) {
            lastSensorUpdateTimer.reset();
            lastSensorUpdateTimer.start();
        }
            
        lastSensorRed = tower.getDetectedColor().red;

        if (!rotationArms.getKilled() && rotationArms.atGoal() && rotationArms.getGoal() == ClimberConstants.intakePositionRad) {

            //Need to add ir sensing, should have a (mostly) constant value for no ball, need to check if there is a ball before everything else
            boolean wrongBall = //(tower.getDetectedColor().red > 60 && DriverStation.getAlliance() == Alliance.Blue) ||
                                (tower.getDetectedColor().red < 5 && DriverStation.getAlliance() == Alliance.Red);

            if (wrongBall) {// && lastSensorUpdateTimer.get() < 0.1) {
                reverseTimer.reset();
                reverseTimer.start();
            }

            if (reverseTimer.get() > 0 && reverseTimer.get() < 50) {
                tower.setConveyorPercent(-BallConstants.towerPower);
                tower.setIndexWheelsPercent(-BallConstants.towerPower);
                intake.setPercent(BallConstants.intakePower.get());
            }
            else {

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