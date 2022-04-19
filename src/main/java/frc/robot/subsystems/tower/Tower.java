package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PicoColorSensor.RawColor;
import frc.robot.RobotState;

public class Tower extends SubsystemBase {

    public enum BallType {
        None, Red, Blue;

        public static byte toByte(BallType type) { return (byte)(type == BallType.None ? 0 : type == BallType.Blue ? 1 : 2); }
    }

    //private SPI spi = new SPI(Port.kMXP);

    private final TowerIO io;
    private final TowerIOInputs ioInputs = new TowerIOInputs();

    private double lastConveyorPercent = 0;
    private double lastWheelPercent = 0;

    private static BallType topBall = BallType.None;
    private static BallType bottomBall = BallType.None;

    private int ballCount = 1;

    private boolean prevTopSensorState = false;
    private boolean currentTopSensorState = false;
    
    private boolean prevBottomSensorState = false;
    private boolean currentBottomSensorState = false;

    private BallType lastShotColor = BallType.None;

    public Tower(TowerIO io) {

        this.io = io;

        topBall = DriverStation.getAlliance() == Alliance.Red ? BallType.Red : BallType.Blue;

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

        Logger.getInstance().recordOutput("Tower/TopBall", BallType.toByte(topBall));
        Logger.getInstance().recordOutput("Tower/BottomBall", BallType.toByte(bottomBall));
        Logger.getInstance().recordOutput("Tower/BallCount", (byte)ballCount);

        prevTopSensorState = currentTopSensorState;
        prevBottomSensorState = currentBottomSensorState;

    }
    
    private BallType getBottomSensorColor() {
        if (ioInputs.detectedColor.red > 110)
            return BallType.Red;
        if (ioInputs.detectedColor.green > 35)
            return BallType.Blue;
        return BallType.None;
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

    public static BallType getTopBall() {
        return topBall;
    }

    public static BallType getBottomBall() {
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

    /*public static void setError(boolean e) {
        error = e;
    }*/
    
}
