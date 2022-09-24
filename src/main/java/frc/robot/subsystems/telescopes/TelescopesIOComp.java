package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

public class TelescopesIOComp implements TelescopesIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final TimeOfFlight leftLidar;
    private final TimeOfFlight rightLidar;

    public TelescopesIOComp() {

        // Initialization
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANDevices.rightTelescopingMotorID, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftLidar = new TimeOfFlight(CANDevices.leftLidar);
        rightLidar = new TimeOfFlight(CANDevices.rightLidar);

        // Options
        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        
        leftEncoder.setPositionConversionFactor(ClimberConstants.telescopeConversionFactor);
        rightEncoder.setPositionConversionFactor(ClimberConstants.telescopeConversionFactor);

        leftLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        rightLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);

    }

    @Override
    public void updateInputs(TelescopesIOInputs inputs) {

        if (leftLidar.getStatus() == TimeOfFlight.Status.Valid)
            inputs.leftLidarPositionM = leftLidar.getRange() - ClimberConstants.telescopeOffsetM;
        if (rightLidar.getStatus() == TimeOfFlight.Status.Valid)
            inputs.rightLidarPositionM = rightLidar.getRange() - ClimberConstants.telescopeOffsetM;

        inputs.leftEncoderPositionM = leftEncoder.getPosition();
        inputs.rightEncoderPositionM = rightEncoder.getPosition();

        inputs.leftVelocityMPerS = leftEncoder.getVelocity() / 60.0;
        inputs.rightVelocityMPerS = rightEncoder.getVelocity() / 60.0;

        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();

        Logger.getInstance().recordOutput("Telescopes/CurrentDraw", leftMotor.getOutputCurrent());

    }

    @Override
    public void setLeftEncoder(double positionM) {
        leftEncoder.setPosition(positionM);
    }
    
    @Override
    public void setRightEncoder(double positionM) {
        rightEncoder.setPosition(positionM);
    }

    @Override
    public void setLeftVolts(double volts) {
        leftMotor.setVoltage(volts);        
    }

    @Override
    public void setRightVolts(double volts) {
        rightMotor.setVoltage(volts);        
    }

    @Override
    public double getLeftCurrentDraw() {
        return leftMotor.getOutputCurrent();
    }
    
    @Override
    public double getRightCurrentDraw() {
        return rightMotor.getOutputCurrent();
    }

    @Override
    public boolean isLeftLidarValid() {
        return leftLidar.getStatus() == TimeOfFlight.Status.Valid;
    }

    @Override
    public boolean isRightLidarValid() {
        return rightLidar.getStatus() == TimeOfFlight.Status.Valid;
    }
    
}