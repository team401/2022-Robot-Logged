package frc.robot.subsystems.rotationArms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ClimberConstants;

public class RotationArmsIOComp implements RotationArmsIO {

    private final TalonSRX leftMotor;
    private final TalonSRX rightMotor;

    private final DutyCycleEncoder leftEncoder;
    private final DutyCycleEncoder rightEncoder;

    private final ProfiledPIDController leftController = new ProfiledPIDController(3.5, 0, 0.1, 
        new TrapezoidProfile.Constraints(0, 0));

    private final ProfiledPIDController rightController = new ProfiledPIDController(3.5, 0, 0.1, 
        new TrapezoidProfile.Constraints(0, 0));

    private boolean killed = false;

    private static void setFramePeriods(TalonSRX talon, boolean needMotorSensor) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        //if (!needMotorSensor) {
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        //}
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);
    }

    public RotationArmsIOComp(int leftMotorID, int rightMotorID, int leftEncoderID, int rightEncoderID) {
        
        leftMotor = new TalonSRX(leftMotorID);
        rightMotor = new TalonSRX(rightMotorID);

        leftEncoder = new DutyCycleEncoder(leftEncoderID);
        rightEncoder = new DutyCycleEncoder(rightEncoderID);

        setFramePeriods(leftMotor, false);
        setFramePeriods(rightMotor, false);

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftEncoder.setDistancePerRotation(2 * Math.PI);
        rightEncoder.setDistancePerRotation(2 * Math.PI);

        leftMotor.configVoltageCompSaturation(12, 1000);
        leftMotor.enableVoltageCompensation(true);
        rightMotor.configVoltageCompSaturation(12, 1000);
        rightMotor.enableVoltageCompensation(true);

        leftMotor.configNeutralDeadband(0, 1000);
        rightMotor.configNeutralDeadband(0, 1000);

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
        leftMotor.set(ControlMode.PercentOutput, volts/12);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.set(ControlMode.PercentOutput, volts/12);
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