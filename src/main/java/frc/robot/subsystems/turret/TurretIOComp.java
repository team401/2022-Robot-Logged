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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.CANDevices;

public class TurretIOComp implements TurretIO {

    private final TalonFX turretMotor = new TalonFX(CANDevices.turretMotorID);
    private final CANCoder turretEncoder = new CANCoder(CANDevices.turretEncoderID);

    public TurretIOComp() {
       turretMotor.configFactoryDefault(1000);
       turretMotor.setNeutralMode(NeutralMode.Brake);
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
        inputs.positionRad = Units.degreesToRadians(turretEncoder.getPosition());
        inputs.velocityRadPerS = Units.degreesToRadians(turretEncoder.getVelocity());
        inputs.current = turretMotor.getSupplyCurrent();
    }

    @Override
    public void resetEncoderAbsolute() {
        // The absolute position is configured to wrap at the crossing between 0 and 360, as an unsigned value.
        // This means we can boot up at either side of that crossing point, meaning if we subtract our offset on one side
        // we get a negative number, and on the other we get a positive number.  We want to re-center this range
        // so that zero points straight ahead, but we also don't jump from positive to negative at 180 degrees.

        // This is set up to read the absolute angle, subtract the offset, and then reconfigure the relative encoder
        // to use the "angle modulus" of the new offset angle.  This puts zero at forward, and once the relative encoder
        // is set up it will not wrap around.
        double absoluteOffset = Units.degreesToRadians(turretEncoder.getAbsolutePosition()) - TurretConstants.turretEncoderOffsetRad;
        turretEncoder.setPosition(Units.radiansToDegrees(MathUtil.angleModulus(absoluteOffset)));
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

    public void setNeutralMode(NeutralMode mode) {
        turretMotor.setNeutralMode(mode);
    }
}
