package frc.robot.subsystems.ledManager;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.Tower.BallType;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;

public class LEDManager extends SubsystemBase {

    /**
    * FORMAT:
    * byte - field relative rotation (deg)
    * byte - top ball (0=none, 1=blue, 2=red)
    * byte - bottom ball (same as above)
    * byte - lock (0=no, 1=yes)
    * byte - error (0=no, 1=yes)
    */

    private static boolean error = false;

    private AddressableLED leftLed;
    private AddressableLED rightLed;

    private AddressableLEDBuffer leftBuffer;
    private AddressableLEDBuffer rightBuffer;

    private int[][] leftFront = new int[3][3];
    private int[][] leftMid = new int[3][3];
    private int[][] leftBack = new int[3][3];

    private int[][] rightFront = new int[3][3];
    private int[][] rightMid = new int[3][3];
    private int[][] rightBack = new int[3][3];


    public LEDManager(int leftPort, int rightPort) {

        leftLed = new AddressableLED(leftPort);
        rightLed = new AddressableLED(rightPort);

        leftBuffer = new AddressableLEDBuffer(9);
        rightBuffer = new AddressableLEDBuffer(9);

        leftLed.setLength(leftBuffer.getLength());
        rightLed.setLength(rightBuffer.getLength());

        leftLed.start();
        rightLed.start();
    }

    @Override
    public void periodic() {

        updateStrip(leftFront);
        updateStrip(leftMid);
        updateStrip(leftBack);

        updateStrip(rightFront);
        updateStrip(rightMid);
        updateStrip(rightBack);

        updateRotation();

        leftLed.setData(leftBuffer);
        rightLed.setData(rightBuffer);

    }

    private void updateStrip(int[][] strip) {

        /**
         * locked and revved - white
         * two correct balls - green
         * one correct balls - yellow
         * has incorrect ball - red
         * error - random
         */

        int topBall = BallType.toByte(Tower.getTopBall());
        int bottomBall = BallType.toByte(Tower.getBottomBall());
        boolean readyToShoot = Math.abs(Vision.getTX()) < 1 && Shooter.atGoalStatic();
        int alliance = DriverStation.getAlliance() == Alliance.Blue ? 1 : 2;
        
        int color[] = {0, 0, 0};

        if (readyToShoot) 
            color = new int[]{255, 255, 255};
        else if (topBall == bottomBall && topBall == alliance)  
            color = new int[]{0, 255, 0};
        else if (topBall == alliance) 
            color = new int[]{255, 255, 0};
        else if ((topBall != 0 && topBall != alliance) || (bottomBall != 0 && bottomBall != alliance))
            color = new int[]{255, 0, 0};
        else if (error)
            color = new int[]{(int)(Math.random()*256), (int)(Math.random()*256), (int)(Math.random()*256)};

        for (int i = 0; i < strip.length; i++) {
            strip[i][0] = color[0];
            strip[i][1] = color[1];
            strip[i][2] = color[2];
        }

    }

    private void updateRotation() {
        double deg = RobotState.getInstance().getLatestFieldToVehicle().getRotation().getDegrees();

        // Positives
        if (deg < 45 && deg > 0) {
            // left back
        }
        else if (deg < 90 && deg > 45) {
            // left back and left mid
        }
        else if (deg < 135 && deg > 90) {
            // left mid and left front
        }
        else if (deg < 180 && deg > 135) {
            // left front
        }

        // Negatives
        else if (deg > -45 && deg < 0) {
            // right back
        }
        else if (deg > -90 && deg < -45) {
            // right back and right mid
        }
        else if (deg > -135 && deg < -90) {
            // right mid and right front
        }
        else if (deg > -180 && deg < -135) {
            // right front
        }
        

        /*if (deg < 22.5 && deg > -22.5) {
            blankStrip(leftFront);
            blankStrip(leftMid);
            blankStrip(rightFront);
            blankStrip(rightMid);
        }
        else if (deg < 67.5 && deg > 22.5) {
            blankStrip(leftFront);
            blankStrip(rightFront);
            blankStrip(rightMid);
            blankStrip(rightBack);
        }
        else if (deg < 112.5 && deg > 67.5) {
            blankStrip(leftFront);
            blankStrip(leftBack);
            blankStrip(rightFront);
            blankStrip(rightMid);
            blankStrip(rightBack);
        }
        */ 

    }

    private void blankStrip(int[][] strip) {
        for (int i = 0; i < strip.length; i++) {
            for (int j = 0; j < strip[0].length; j++) {
                strip[i][j] = 0;
            }
        }
    }

    public static void setError(boolean e) {
        error = e;
    }
    
}