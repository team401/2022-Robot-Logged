package frc.robot.subsystems.rotationArms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

    }

    @Override
    public void updateInputs(RotationArmsIOInputs inputs) {
        inputs.leftPositionRad = leftEncoder.getDistance() * 2.0 * Math.PI - ClimberConstants.leftRotationOffset;
        inputs.rightPositionRad = rightEncoder.getDistance() * 2.0 * Math.PI - ClimberConstants.rightRotationOffset;
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
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