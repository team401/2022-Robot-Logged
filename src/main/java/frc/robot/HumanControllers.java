package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class HumanControllers {
    
    private final Joystick leftStick;
    private final Joystick rightStick;
    private final XboxController gamepad;

    public HumanControllers(Joystick left, Joystick right, XboxController gamepad) {

        this.leftStick = left;
        this.rightStick = right;
        this.gamepad = gamepad;
    }

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
