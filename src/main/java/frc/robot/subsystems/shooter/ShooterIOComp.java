package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOComp implements ShooterIO {
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;

    private final CANSparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkMaxPIDController hoodController;
    
    public ShooterIOComp() {
        leftShooterMotor = new TalonFX(CANDevices.leftShooterMotorID, Constants.canivoreName);
        rightShooterMotor = new TalonFX(CANDevices.rightShooterMotorID, Constants.canivoreName);

        hoodMotor = new CANSparkMax(CANDevices.hoodMotorID, MotorType.kBrushless);

        rightShooterMotor.configFactoryDefault();
        leftShooterMotor.configFactoryDefault();

        rightShooterMotor.configNeutralDeadband(0);
        leftShooterMotor.configNeutralDeadband(0);

        hoodMotor.restoreFactoryDefaults();

        rightShooterMotor.setInverted(TalonFXInvertType.OpposeMaster);
        hoodMotor.setInverted(true);

        rightShooterMotor.follow(leftShooterMotor);
        rightShooterMotor.setStatusFramePeriod(1, 255);
        rightShooterMotor.setStatusFramePeriod(2, 255);

        leftShooterMotor.configVoltageCompSaturation(12, 1000);
        leftShooterMotor.enableVoltageCompensation(true);

        // Current Limits
        hoodMotor.setSmartCurrentLimit(30);

        // sets up hood PID Controller
        hoodController = hoodMotor.getPIDController();

        hoodEncoder = hoodMotor.getEncoder();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelSpeedRadPerS = leftShooterMotor.getSelectedSensorVelocity() * 2.0 * Math.PI * 10.0 / 2048.0;
        inputs.hoodPositionRad = hoodEncoder.getPosition() * 2.0 * Math.PI / ShooterConstants.hoodRackRatio
                + ShooterConstants.hoodOffsetRad;
        inputs.flywheelCurrent = new double[] { leftShooterMotor.getSupplyCurrent(),
                rightShooterMotor.getSupplyCurrent() };
        inputs.hoodCurrent = hoodMotor.getOutputCurrent();
        inputs.hoodVelocity = hoodEncoder.getVelocity();
    }

    @Override
    public void zeroHoodEncoder() {
        hoodEncoder.setPosition(0);

    }

    @Override
    public void setHoodPositionSetpoint(double angleRad) {
        double motorRevs = (angleRad - ShooterConstants.hoodOffsetRad) / 2.0 / Math.PI * ShooterConstants.hoodRackRatio;
        hoodController.setReference(motorRevs, ControlType.kPosition);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    @Override
    public void setHoodPD(double p, double d) {
        hoodController.setP(p);
        hoodController.setD(d);
    }

    @Override
    public void setFlywheelVelocity(double velocityRadPerS, double ffVolts) {
        double flywheelTicksPer100ms = velocityRadPerS / 2.0 / Math.PI / 10.0 * 2048.0;
        leftShooterMotor.set(ControlMode.Velocity, flywheelTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        leftShooterMotor.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void setFlywheelPD(double p, double d) {
        leftShooterMotor.config_kP(0, p, 1000);
        leftShooterMotor.config_kD(0, d, 1000);
    }

}