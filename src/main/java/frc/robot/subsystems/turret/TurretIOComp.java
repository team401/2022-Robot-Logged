package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.CANDevices;

public class TurretIOComp implements TurretIO {

    private final TalonFX turretMotor = new TalonFX(CANDevices.turretMotorID);
    private final CANCoder turretEncoder = new CANCoder(CANDevices.turretEncoderID);

    public TurretIOComp() {
       turretMotor.configFactoryDefault(1000);
       turretMotor.setNeutralMode(NeutralMode.Coast);
       turretMotor.setInverted(true);
       turretMotor.configVoltageCompSaturation(12, 1000);
       turretMotor.enableVoltageCompensation(true);
       turretMotor.configNeutralDeadband(0, 1000);
       setFramePeriods(turretMotor);

       turretEncoder.configFactoryDefault(1000);
       turretEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 1000);
       turretEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);
       turretEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, 1000);
    }

    private static void setFramePeriods(TalonFX talon) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
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


    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRad = Units.degreesToRadians(turretEncoder.getPosition()) - TurretConstants.turretEncoderOffsetRad;
        inputs.velocityRadPerS = Units.degreesToRadians(turretEncoder.getVelocity());
        inputs.current = turretMotor.getSupplyCurrent();
    }

    @Override
    public void resetEncoderAbsolute() {
        turretEncoder.setPositionToAbsolute();
    }

    @Override
    public void setVoltage(double voltage) {
        turretMotor.set(ControlMode.PercentOutput, voltage/12);
    }
        

    @Override
    public void setVelocitySetpoint(double velocityRadPerS, double ffVolts) {
        double velocityTicksPer100ms = velocityRadPerS / 10.0 / 2.0 / Math.PI * TurretConstants.turretGearRatio;
        turretMotor.set(ControlMode.Velocity, velocityTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12);
    }

    @Override
    public void setVelocityPD(double p, double d) {
        turretMotor.config_kP(0, p, 1000);
        turretMotor.config_kD(0, d, 1000);
    }
}
