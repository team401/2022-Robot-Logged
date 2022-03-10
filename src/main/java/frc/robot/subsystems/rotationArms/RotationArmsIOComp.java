package frc.robot.subsystems.rotationArms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ClimberConstants;

public class RotationArmsIOComp implements RotationArmsIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final DutyCycleEncoder leftEncoder;
    private final DutyCycleEncoder rightEncoder;

    private final ProfiledPIDController leftController = new ProfiledPIDController(3.5, 0, 0.1, 
        new TrapezoidProfile.Constraints(0, 0));

    private final ProfiledPIDController rightController = new ProfiledPIDController(3.5, 0, 0.1, 
        new TrapezoidProfile.Constraints(0, 0));

    private boolean killed = false;

    public RotationArmsIOComp(int leftMotorID, int rightMotorID, int leftEncoderID, int rightEncoderID) {
        
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);

        leftEncoder = new DutyCycleEncoder(leftEncoderID);
        rightEncoder = new DutyCycleEncoder(rightEncoderID);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftEncoder.setDistancePerRotation(2 * Math.PI);
        rightEncoder.setDistancePerRotation(2 * Math.PI);

        leftMotor.enableVoltageCompensation(12.0);
        rightMotor.enableVoltageCompensation(12.0);

        //can't do this for Spark Maxes
        //leftMotor.configNeutralDeadband(0, 1000);
        //rightMotor.configNeutralDeadband(0, 1000);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

    }

    private double getLeftPositionRad() {
        if(leftEncoder.getDistance() + ClimberConstants.leftRotationOffset > Math.PI)
            return leftEncoder.getDistance() + ClimberConstants.leftRotationOffset - 2 * Math.PI;
        return (leftEncoder.getDistance() + ClimberConstants.leftRotationOffset);
    }

    private double getRightPositionRad() {
        if(rightEncoder.getDistance() + ClimberConstants.rightRotationOffset > Math.PI)
            return rightEncoder.getDistance() + ClimberConstants.rightRotationOffset - 2 * Math.PI;
        return (rightEncoder.getDistance() + ClimberConstants.rightRotationOffset);
    }

    @Override
    public void updateInputs(RotationArmIOInputs inputs) {
        inputs.leftPositionRad = getLeftPositionRad();
        inputs.rightPositionRad = getRightPositionRad();
    }

    @Override
    public void resetControllers() {
        leftController.reset(getLeftPositionRad());
        rightController.reset(getRightPositionRad());
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.set(volts/12);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.set(volts/12);
    }

    @Override
    public void setLeftDesiredPositionRad(double desiredRadians) {
        double output = leftController.calculate(getLeftPositionRad(), desiredRadians);
        if (!killed)
            setLeftVoltage(output);
    }

    @Override
    public void setRightDesiredPositionRad(double desiredRadians) {
        double output = rightController.calculate(getRightPositionRad(), desiredRadians);
        if (!killed)
            setRightVoltage(output);
    }

    @Override
    public void kill() {
        setLeftVoltage(0);
        setLeftVoltage(0);
        killed = true;
    }

    @Override
    public void setPD(double p, double d) {
        leftController.setP(p);
        leftController.setP(d);

        rightController.setP(p);
        rightController.setP(d);
    }

}