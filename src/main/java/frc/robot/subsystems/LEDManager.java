package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOChannels;

public class LEDManager extends SubsystemBase {

    /**
     * FORMAT:
     * int - field relative rotation (deg)
     * int - top ball (0=none, 1=blue, 2=red)
     * int - bottom ball (same as above)
     * int - lock (0=no, 1=yes)
     * int - error (0=no, 1=yes)
     */

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int rainbowFirstPixelHue;

    public LEDManager(int port) {

        led = new AddressableLED(port);

        ledBuffer = new AddressableLEDBuffer(60);
        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        if (!DriverStation.isEnabled())
            setRainbow();
        
        led.setData(ledBuffer);
    }

    public void setColor(int r, int g, int b) {

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }

    }

    public void setRainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }
    
}
