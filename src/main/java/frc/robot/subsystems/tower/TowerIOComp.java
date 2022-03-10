package frc.robot.subsystems.tower;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;

public class TowerIOComp implements TowerIO {

    private final CANSparkMax conveyorMotor;
    private final CANSparkMax indexMotor;

    private final DigitalInput bottomBanner;
    private final DigitalInput topBanner;

    public TowerIOComp() {
        conveyorMotor = new CANSparkMax(CANDevices.conveyorMotorID, MotorType.kBrushed);
        indexMotor = new CANSparkMax(CANDevices.indexMotorID, MotorType.kBrushed);
        bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
        topBanner = new DigitalInput(DIOChannels.bottomBannerPort);

        conveyorMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setIdleMode(IdleMode.kCoast);

        conveyorMotor.setInverted(true);

    }

    @Override
    public void updateInputs(TowerIOInputs inputs) {

        inputs.bottomState = !bottomBanner.get();
        inputs.topState = !topBanner.get();
        
    }

    @Override
    public void setConveyorPercent(double percent) {
        conveyorMotor.set(percent);
    }

    @Override
    public void setIndexWheelsPercent(double percent) {
        indexMotor.set(percent);
    }

}
