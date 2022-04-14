package frc.robot.subsystems.tower;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PicoColorSensor.RawColor;

public class Tower extends SubsystemBase {

    public enum BallType {
        None, Red, Blue
    }

    private SPI spi;

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

    private BallType lastShotColor = BallType.None;

    private static boolean error = false;

    public Tower(TowerIO io) {

        this.io = io;

        spi = new SPI(Port.kMXP);

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
            lastShotColor = topBall;
            if (ballCount == 1) {
                topBall = BallType.None;
            }
            else if (ballCount == 2) {
                topBall = bottomBall;
                bottomBall = BallType.None;;
            }
            ballCount--;
        }

        // Anti-Shooting
        if (lastConveyorPercent < 0 && !prevTopSensorState && currentTopSensorState) {
            if (ballCount == 1) {
                bottomBall = topBall;
                topBall = lastShotColor;
            }
            else if (ballCount == 0) {
                topBall = lastShotColor;
            }

            ballCount++;
        }

        prevTopSensorState = currentTopSensorState;
        prevBottomSensorState = currentBottomSensorState;

        /**
         * FORMAT:
         * byte - field relative rotation (deg)
         * byte - top ball (0=none, 1=blue, 2=red)
         * byte - bottom ball (same as above)
         * byte - lock (0=no, 1=yes)
         * byte - error (0=no, 1=yes)
         */
        
        byte[] data = {
            (byte)(frc.robot.RobotState.getInstance().getLatestFieldToVehicle().getRotation().getDegrees()/10),
            (byte)(topBall == BallType.None ? 0 : topBall == BallType.Blue ? 1 : 2),
            (byte)(bottomBall == BallType.None ? 0 : bottomBall == BallType.Blue ? 1 : 2),
            (byte)(Math.abs(Vision.getTX()) < 1 ? 1 : 0),
            (byte)(error ? 1 : 0)
        };
        spi.transaction(data, null, data.length);

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
        lastWheelPercent = percent;
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

    public static void setError(boolean e) {
        error = e;
    }
    
}
