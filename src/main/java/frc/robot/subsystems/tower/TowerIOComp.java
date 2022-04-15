package frc.robot.subsystems.tower;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.util.PicoColorSensor;

public class TowerIOComp implements TowerIO {

    private final CANSparkMax conveyorMotor;
    private final CANSparkMax indexMotor;

    private final DigitalInput topBanner;

    // PicoColorSensor initialization needs to be here
    private final PicoColorSensor colorSensor = new PicoColorSensor();

    public TowerIOComp() {
        conveyorMotor = new CANSparkMax(CANDevices.conveyorMotorID, MotorType.kBrushed);
        indexMotor = new CANSparkMax(CANDevices.indexMotorID, MotorType.kBrushed);
        topBanner = new DigitalInput(DIOChannels.topBannerPort);

        conveyorMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setIdleMode(IdleMode.kBrake);

        conveyorMotor.enableVoltageCompensation(12);
        indexMotor.enableVoltageCompensation(12);

        conveyorMotor.setSmartCurrentLimit(30);
        indexMotor.setSmartCurrentLimit(20);

        conveyorMotor.setInverted(false);
        indexMotor.setInverted(true);

        indexMotor.burnFlash();
        conveyorMotor.burnFlash();
    }

    @Override
    public void updateInputs(TowerIOInputs inputs) {
        inputs.topSensor = !topBanner.get();
        inputs.conveyorCurrent = conveyorMotor.getOutputCurrent();
        inputs.indexCurrent = indexMotor.getOutputCurrent();
        inputs.detectedColor = colorSensor.getRawColor0();
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
