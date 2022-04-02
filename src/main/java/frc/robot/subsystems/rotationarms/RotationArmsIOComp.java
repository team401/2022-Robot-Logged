package frc.robot.subsystems.rotationarms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DIOChannels;

public class RotationArmsIOComp implements RotationArmsIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final DutyCycleEncoder leftEncoder;
    private final DutyCycleEncoder rightEncoder;

    public RotationArmsIOComp() {
        
        leftMotor = new CANSparkMax(CANDevices.leftRotationMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(CANDevices.rightRotationMotorID, MotorType.kBrushed);

        leftEncoder = new DutyCycleEncoder(DIOChannels.leftRotationArmEncoder);
        rightEncoder = new DutyCycleEncoder(DIOChannels.rightRotationArmEncoder);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        rightEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);

        setStatusFrames(leftMotor);
        setStatusFrames(rightMotor);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        //leftMotor.burnFlash();
        //rightMotor.burnFlash();

    }

    private static void setStatusFrames(CANSparkMax spark) {
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    }

    @Override
    public void updateInputs(RotationArmsIOInputs inputs) {
        inputs.leftPositionRad = leftEncoder.get() * 2.0 * Math.PI - ClimberConstants.leftRotationOffset;
        inputs.rightPositionRad = rightEncoder.get() * 2.0 * Math.PI - ClimberConstants.
        rightRotationOffset;
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
    }

    @Override
    public void resetEncoder() {
        leftEncoder.reset();
        rightEncoder.reset();
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