package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterIOComp implements ShooterIO {

    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;

    private final CANSparkMax hoodMotor;

    private final RelativeEncoder hoodEncoder;

    private final SparkMaxPIDController hoodController;

    public ShooterIOComp(int leftShooterID, int rightShooterID, int hoodID) {
        leftShooterMotor = new TalonFX(leftShooterID);
        rightShooterMotor = new TalonFX(rightShooterID);

        hoodMotor = new CANSparkMax(hoodID, MotorType.kBrushless);

        rightShooterMotor.configFactoryDefault();
        leftShooterMotor.configFactoryDefault();

        rightShooterMotor.configNeutralDeadband(0.001);
        leftShooterMotor.configNeutralDeadband(0.001);

        rightShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        hoodMotor.restoreFactoryDefaults();

        leftShooterMotor.config_kF(0, 1023.0/21650, 10);
        leftShooterMotor.config_kP(0, 0.1, 10);
        leftShooterMotor.config_kI(0, 0, 10);
        leftShooterMotor.config_kD(0, 0, 10);

        rightShooterMotor.setInverted(true);
        hoodMotor.setInverted(true);

        rightShooterMotor.follow(leftShooterMotor);
        rightShooterMotor.setStatusFramePeriod(1, 255);
        rightShooterMotor.setStatusFramePeriod(2, 255);

        //Current Limits
        hoodMotor.setSmartCurrentLimit(20);

        //Soft Limits
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        
        //sets up hood PID Controller
        hoodController = hoodMotor.getPIDController();
        hoodController.setP(0.1);
        hoodController.setI(0);
        hoodController.setD(0);

        hoodController.setSmartMotionMaxVelocity(6, 0);
        hoodController.setSmartMotionMaxAccel(20, 0);

        hoodEncoder = hoodMotor.getEncoder();

        hoodEncoder.setPositionConversionFactor(1.0/2048);
    }

    public double getShooterRPM() {
        return leftShooterMotor.getSelectedSensorVelocity() / 2048 * 600;
    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public double getHoodVelocity() {
        return hoodEncoder.getVelocity();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterRPM = getShooterRPM();
        inputs.hoodPosition = getHoodPosition();
        inputs.hoodVelocity = getHoodVelocity();
    }

    @Override
    public void zeroHoodEncoder() {
        hoodEncoder.setPosition(0);
    }

    @Override
    public void hoodSetPosition(double position) {
        
    }

    @Override
    public void hoodSetPercent(double percent) {
        hoodMotor.set(percent);
    }

    @Override
    public void setHoodSoftLimits(float forward, float reverse) {
        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forward);
        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverse);
    }

    @Override
    public void setShooterPercent(double percent) {
        leftShooterMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setShooterSpeed(double desiredRPM) {
        
    }

    @Override
    public void setHoodPD(double p, double d) {
        
    }

    @Override
    public void setShooterPD(double p, double d) {
        
    }

    @Override
    public void setTrapezoidalConstraints(double velocity, double acceleration) {
        
    }
    
}