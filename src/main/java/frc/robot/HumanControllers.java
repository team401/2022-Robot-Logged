package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class HumanControllers {
    
    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);


    public int getRightTriggerValue() {

        double value = gamepad.getRightTriggerAxis();

        if (value >= 0.3) 
            return 1;
        return 0;

    }

    public int getLeftTriggerValue() {

        double value = gamepad.getLeftTriggerAxis();

        if (value >= 0.3) 
            return 1;
        return 0;
        

    }

}
