package frc.robot.subsystems.ledManager;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.Tower.BallType;
import frc.robot.subsystems.vision.Vision;

public class LEDManager extends SubsystemBase {

    private static boolean error = false;

    private AddressableLED leftLed;
    private AddressableLED rightLed;

    private AddressableLEDBuffer leftBuffer;
    private AddressableLEDBuffer rightBuffer;

    private final int ledCountPerSide = 34;
    private final int ledArmCount = 22;

    private int rainbowFirstPixelHue = 0;

    public LEDManager(int leftPort, int rightPort) {

        leftLed = new AddressableLED(leftPort);
        rightLed = new AddressableLED(rightPort);

        leftBuffer = new AddressableLEDBuffer(ledCountPerSide);
        rightBuffer = new AddressableLEDBuffer(ledCountPerSide);

        leftLed.setLength(leftBuffer.getLength());
        rightLed.setLength(rightBuffer.getLength());

        leftLed.start();
        rightLed.start();
    }

    @Override
    public void periodic() {

        // Blank buffers
        for (int i = 0; i < leftBuffer.getLength(); i++)
            leftBuffer.setRGB(i, 0, 0, 0);
        for (int i = 0; i < rightBuffer.getLength(); i++)
            rightBuffer.setRGB(i, 0, 0, 0);
        
        if (DriverStation.isEnabled())
            updateStrips();
        else
            rainbowClimbers();
        
        leftLed.setData(leftBuffer);
        rightLed.setData(rightBuffer);

    }

    private void updateStrips() {

        double deg = RobotState.getInstance().getLatestFieldToVehicle().getRotation().getDegrees();

        if (deg > 67.5 || deg < -112.5) // Left front
            updateStrip(leftBuffer, ledArmCount);
		if (deg > 22.5 || deg < -157.5) // Left mid front
            updateStrip(leftBuffer, ledArmCount+3);
		if (deg > -22.5 && deg < 157.5) // Left mid back
            updateStrip(leftBuffer, ledArmCount+6);
		if (deg > -67.5 && deg < 112.5) // Left back
            updateStrip(leftBuffer, ledArmCount+9);
        
		if (deg > 112.5 || deg < -67.5) // Right front
            updateStrip(rightBuffer, ledArmCount);
		if (deg > 157.5 || deg < -22.5) // Right mid front
            updateStrip(rightBuffer, ledArmCount+3);
		if (deg > -157.5 && deg < 22.5) // Right mid back
            updateStrip(rightBuffer, ledArmCount+6);
		if (deg > -112.5 && deg < 67.5) // Right back
            updateStrip(rightBuffer, ledArmCount+9);

    }

    private void updateStrip(AddressableLEDBuffer buffer, int offset) {

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

        Color color = Color.kBlack;

        if (readyToShoot) 
            color = Color.kWhite;
        else if (topBall == bottomBall && topBall == alliance)  
            color = Color.kGreen;
        else if (topBall == alliance) 
            color = Color.kYellow;
        else if ((topBall != 0 && topBall != alliance) || (bottomBall != 0 && bottomBall != alliance))
            color = Color.kRed;
        else if (error)
            color = new Color((int)(Math.random()*256), (int)(Math.random()*256), (int)(Math.random()*256));

        for (int i = offset; i < offset+3; i++)
            buffer.setLED(i, color);

    }

    private void rainbowClimbers() {
        for (int i = 0; i < ledArmCount; i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / ledArmCount)) % 180;
            leftBuffer.setHSV(i, hue, 255, 128);
            rightBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    public static void setError(boolean e) {
        error = e;
    }
    
}