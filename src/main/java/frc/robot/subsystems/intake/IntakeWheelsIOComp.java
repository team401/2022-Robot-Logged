package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CANDevices;

public class IntakeWheelsIOComp implements IntakeWheelsIO {

    private final CANSparkMax intakeMotor;

    public IntakeWheelsIOComp() {
        intakeMotor = new CANSparkMax(CANDevices.intakeMotorID, MotorType.kBrushed);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
    }

    @Override
    public void updateInputs(IntakeWheelsIOInput inputs) {
        inputs.current = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setPercent(double percent) {
        intakeMotor.set(percent);        
    }
    
}
