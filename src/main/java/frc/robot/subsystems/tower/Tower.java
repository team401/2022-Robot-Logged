package frc.robot.subsystems.tower;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;
import frc.robot.util.PicoColorSensor.RawColor;

public class Tower extends SubsystemBase {

    public enum BallType {
        None, Red, Blue
    }

    private final TowerIO io;
    private final TowerIOInputs ioInputs = new TowerIOInputs();

    private double lastConveyorPercent = 0;
    private double lastWheelPercent = 0;

    private BallType topBall = BallType.None;
    private BallType bottomBall = BallType.None;

    private int ballCount = 1;

    private boolean prevTopSensorState = false;
    private boolean currentTopSensorState = false;
    
    private boolean prevBottomSensorState = false;
    private boolean currentBottomSensorState = false;

    public Tower(TowerIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {

        io.updateInputs(ioInputs);
        Logger.getInstance().processInputs("Tower", ioInputs);

        currentTopSensorState = getTopSensor();
        currentBottomSensorState = getBottomSensorColor() != BallType.None;

        // Intaking
        if (lastWheelPercent < 0 && prevBottomSensorState && !currentBottomSensorState) {
            if (ballCount == 0)
                topBall = getBottomSensorColor();
            else
                bottomBall = getBottomSensorColor();
            ballCount++;
        }

        // Reverse Intaking
        if (lastWheelPercent > 0 && prevBottomSensorState && !currentBottomSensorState) {
            if (ballCount == 1)
                topBall = BallType.None;
            else if (ballCount == 2)
                bottomBall = BallType.None;
            ballCount--;
        }
        
        // Shooting
        if (lastConveyorPercent > 0 && prevTopSensorState && !currentTopSensorState) {
            if (ballCount == 1) {
                topBall = BallType.None;
            }
            else if (ballCount == 2) {
                topBall = bottomBall;
                bottomBall = BallType.None;;
            }
            ballCount--;
        }

        prevTopSensorState = currentTopSensorState;
        prevBottomSensorState = currentBottomSensorState;

    }

    private BallType getBottomSensorColor() {
        return BallType.None; // TODO: CHANGE
    }

    public void setConveyorPercent(double percent) {
        io.setConveyorPercent(percent);
        lastConveyorPercent = percent;
    }

    public void setIndexWheelsPercent(double percent) {
        io.setIndexWheelsPercent(percent);
    }   

    public boolean getTopSensor() {
        return ioInputs.topSensor;
    }

    public RawColor getDetectedColor() {
        return ioInputs.detectedColor;
    }

    public BallType getTopBall() {
        return topBall;
    }

    public BallType getBottomBall() {
        return bottomBall;
    }

    public int getBallCount() {
        return ballCount;
    }

    public void resetBalls() {
        ballCount = 0;
        topBall = BallType.None;
        bottomBall = BallType.None;
    }
    
}
