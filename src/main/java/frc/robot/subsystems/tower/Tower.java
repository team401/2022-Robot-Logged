package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;
import frc.robot.util.PicoColorSensor.RawColor;
import frc.robot.RobotState;

public class Tower extends SubsystemBase {

    private final TowerIO io;
    private final TowerIOInputs ioInputs = new TowerIOInputs();

    
    private int ballTop = 0;
    private int ballBottom = 0;
    
    private boolean prevTopBannerState = false;
    private int prevColorDetected = 0;
    
    private double prevConveyorPercent = 0;
    
    public Tower(TowerIO io) {
        
        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Tower", ioInputs);

        boolean topBannerState = getTopSensor();
        int color = getDetectedColor();

        // Intake/Reverse Intake
        if (color != 0 && prevColorDetected == 0)
        {
            if (prevConveyorPercent > 0) // Intake
            {
                ballTop = ballBottom;
                ballBottom = color;
            }
            else // Reverse Intake, flush values just in case
            {
                ballBottom = 0;
                ballTop = 0;
            }
        }
        // Shooting
        if (!topBannerState && prevTopBannerState && prevConveyorPercent > 0)
        {
            ballTop = ballBottom;
            ballBottom = 0;
        }

        prevTopBannerState = topBannerState;
        prevColorDetected = color;

        RobotState.getInstance().setCurrentBall(ballTop);

        Logger.getInstance().recordOutput("Tower/BallTop", ballTop);
        Logger.getInstance().recordOutput("Tower/BallBottom", ballBottom);

    }

    public void setConveyorPercent(double percent) {
        io.setConveyorPercent(percent);
        prevConveyorPercent = percent;
    }

    public void setIndexWheelsPercent(double percent) {
        io.setIndexWheelsPercent(percent);
    }   

    public boolean getTopSensor() {
        return ioInputs.topSensor;
    }

    public boolean getBottomSensor() {
        return ioInputs.bottomSensor;
    }

    // TODO: If this ever gets reliable remember to uncomment changes in PrepareToShoot.java
    // 0 = none, 1 = red, 2 = blue
    public int getDetectedColor() {
        RawColor color = ioInputs.detectedColor;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
            return color.red > 40 ? 1 : color.green > 35 ? 2 : 0;
        else
            return color.green > 35 ? 2 : color.red > 40 ? 1 : 0;
    }
    
}
