package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CANDevices;

public class IntakeWheelsIOComp implements IntakeWheelsIO {

    private final CANSparkMax intakeMotor = new CANSparkMax(CANDevices.intakeMotorID, MotorType.kBrushed);

    public IntakeWheelsIOComp() {
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeWheelsIOInput inputs) {
        inputs.current = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setIntakeWheelsPercent(double percent) {
        intakeMotor.set(percent);        
    }
    
}
