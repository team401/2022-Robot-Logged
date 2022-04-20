package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

import org.littletonrobotics.junction.Logger;

public class TelescopesIOComp implements TelescopesIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public TelescopesIOComp() {
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANDevices.rightTelescopingMotorID, MotorType.kBrushless);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        //leftEncoder.setPositionConversionFactor(1 / 5 * 42);
        //leftEncoder.setPositionConversionFactor(factor);

        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

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
        Logger.getInstance().recordOutput("Telescopes/CurrentDraw", leftMotor.getOutputCurrent());
    }

    @Override
    public void resetLeftEncoder() {
        leftEncoder.setPosition(0);      
    }

    @Override
    public void resetRightEncoder() {
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

    @Override
    public double getRightCurrentDraw() {
        return rightMotor.getOutputCurrent();
    }

    @Override
    public double getLeftCurrentDraw() {
        return rightMotor.getOutputCurrent();
    }
    
}
