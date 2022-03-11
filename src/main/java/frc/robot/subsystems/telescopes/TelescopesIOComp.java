package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

public class TelescopesIOComp implements TelescopesIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public TelescopesIOComp() {
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(CANDevices.rightTelescopingMotorID, MotorType.kBrushed);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftEncoder = leftMotor.getEncoder(Type.kQuadrature, 4096);
        rightEncoder = rightMotor.getEncoder(Type.kQuadrature, 4096);

        leftEncoder.setInverted(true);

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        //leftMotor.burnFlash();
        //rightMotor.burnFlash();

    }

    @Override
    public void updateInputs(TelescopesIOInputs inputs) {
        inputs.leftPositionRad = leftEncoder.getPosition() * 2.0 * Math.PI;
        inputs.rightPositionRad = rightEncoder.getPosition() * 2.0 * Math.PI;
        inputs.leftVelocityRadPerS = leftEncoder.getVelocity() * 2.0 * Math.PI / 60.0;
        inputs.rightVelocityRadPerS = rightEncoder.getVelocity() * 2.0 * Math.PI / 60.0;
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
    }

    @Override
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);        
    }

    @Override
    public void setLeftVolts(double volts) {
        leftMotor.setVoltage(volts);        
    }

    @Override
    public void setRightVolts(double volts) {
        rightMotor.setVoltage(volts);        
    }
    
}
