package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CANDevices;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

public class TelescopesIOComp implements TelescopesIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final TimeOfFlight leftLidar;
    private final TimeOfFlight rightLidar;

    public TelescopesIOComp() {
        // Motors
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANDevices.rightTelescopingMotorID, MotorType.kBrushless);
        
        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        // Time of Flight sensors
        leftLidar = new TimeOfFlight(CANDevices.lidarLeft);
        rightLidar = new TimeOfFlight(CANDevices.lidarRight);
        
        leftLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        rightLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);

    }

    @Override
    public void updateInputs(TelescopesIOInputs inputs) {

        // TODO: there is no sensor on the left telescope so just make it the right position and hope
        // when this is fixed also remember to change isLeftLidarOnline()
        inputs.leftPositionM = rightLidar.getRange() - Constants.ClimberConstants.telescopeOffsetM;
        inputs.rightPositionM = rightLidar.getRange() - Constants.ClimberConstants.telescopeOffsetM;
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
        Logger.getInstance().recordOutput("Telescopes/CurrentDraw", leftMotor.getOutputCurrent());
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
    public boolean isLeftLidarOnline() {
        // TODO: change to leftLidar after we get the second TimeOfFlight sensor
        return rightLidar.isRangeValid();
    }

    @Override
    public boolean isRightLidarOnline() {
        return rightLidar.isRangeValid();
    }

    
}
