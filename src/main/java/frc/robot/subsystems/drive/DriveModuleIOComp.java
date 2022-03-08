package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class DriveModuleIOComp implements DriveModuleIO {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX rotationMotor;
    private final CANCoder rotationEncoder;
    private final double initialOffsetRadians;

    private static void setFramePeriods(TalonFX talon) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 10000, 10000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000, 10000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 10000, 10000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10000, 10000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10000, 10000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10000, 10000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10000, 10000);
    }

    public DriveModuleIOComp(        
        int driveMotorID,
        int rotationMotorID,
        int cancoderID,
        double measuredOffsetsRadians
        ) {

        driveMotor = new WPI_TalonFX(driveMotorID);
        rotationMotor = new WPI_TalonFX(rotationMotorID);
        rotationEncoder = new CANCoder(cancoderID);

        driveMotor.configFactoryDefault(10000);
        rotationMotor.configFactoryDefault(10000);
        setFramePeriods(driveMotor);
        setFramePeriods(rotationMotor);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        
        driveMotor.configVoltageCompSaturation(12, 10000);
        driveMotor.enableVoltageCompensation(true);
        rotationMotor.configVoltageCompSaturation(12, 10000);
        rotationMotor.enableVoltageCompensation(true);

        driveMotor.configNeutralDeadband(0, 10000);
        rotationMotor.configNeutralDeadband(0, 10000);

        rotationEncoder.configFactoryDefault(10000);
        
        initialOffsetRadians = measuredOffsetsRadians;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        
        inputs.drivePositionRad = driveMotor.getSelectedSensorPosition() 
            / 2048 * 2 * Math.PI / DriveConstants.driveWheelGearReduction;

        inputs.driveVelocityRadPerS = driveMotor.getSelectedSensorVelocity() 
            / 2048 * 10 * 2 * Math.PI / DriveConstants.driveWheelGearReduction;
            
        //Using relative encoder in the CANCoder
        inputs.rotationPositionRad = Units.degreesToRadians(rotationEncoder.getPosition()) - initialOffsetRadians;

    }

    @Override
    public void zeroEncoders() {
        rotationEncoder.setPositionToAbsolute(1000);
        driveMotor.setSelectedSensorPosition(0, 0, 1000);
    }

    @Override
    public void setRotationVoltage(double volts) {

    }

    @Override
    public void setDriveVoltage(double volts) {
        
    }

    @Override
    public void setDriveVelocity(double velocityRadPerS) {

    }

}