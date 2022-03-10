package frc.robot.subsystems.drive.Turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SuperStructureConstants;

public class TurretIOComp implements TurretIO {

    private final TalonFX turretMotor;
    private final CANCoder turretEncoder;

    public TurretIOComp(int turretMotorID, int turretEncoderID) {

       turretMotor = new TalonFX(turretMotorID);
       turretEncoder = new CANCoder(turretEncoderID);

       turretMotor.configFactoryDefault(1000);
       turretMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
       turretMotor.configRemoteFeedbackFilter(SuperStructureConstants.turretEncoderID, RemoteSensorSource.CANCoder, 0);
       turretMotor.setNeutralMode(NeutralMode.Brake);
       turretMotor.configVoltageCompSaturation(12, 1000);
       turretMotor.enableVoltageCompensation(true);
       turretMotor.configNeutralDeadband(0, 1000);

       turretEncoder.configFactoryDefault(1000);
       turretEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20, 1000);
       turretEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);
       turretEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, 1000);
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
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

        inputs.PositionRad = Units.degreesToRadians(turretEncoder.getAbsolutePosition());
        inputs.VelocityRadPerS = Units.degreesToRadians(turretEncoder.getVelocity());
        
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
    public void setVelocity(double velocityRadPerS, double ffVolts) {

        double velocityRadiansPer100ms = velocityRadPerS / 10;
        turretMotor.set(ControlMode.Velocity, velocityRadiansPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12);
        
    }

    @Override
    public void setSoftLimits() {
        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);

        turretMotor.configForwardSoftLimitThreshold(
            Units.radiansToDegrees(SuperStructureConstants.rightTurretExtremaRadians));
        turretMotor.configReverseSoftLimitThreshold(
            Units.radiansToDegrees(SuperStructureConstants.leftTurretExtremaRadians));
    
    }
    
}
