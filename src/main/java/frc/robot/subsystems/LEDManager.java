package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOChannels;

public class LEDManager extends SubsystemBase {

    /**
     * FORMAT:
     * byte - field relative rotation (deg)
     * byte - top ball (0=none, 1=blue, 2=red)
     * byte - bottom ball (same as above)
     * byte - lock (0=no, 1=yes)
     * byte - error (0=no, 1=yes)
     */

    private SPI spi;

    public LEDManager(int port) {

        spi = new SPI(Port.kOnboardCS0);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {


            
        }

        //spi.transaction(dataToSend, dataReceived, size);
        
    }
    
}
