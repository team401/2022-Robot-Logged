package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ClimberConstants;

public class TelescopesIOComp implements TelescopesIO {

    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    private final ProfiledPIDController leftController = new ProfiledPIDController(1.5, 0.0, 0.0,
            new TrapezoidProfile.Constraints(5, 5));
    private final ProfiledPIDController rightController = new ProfiledPIDController(1.5, 0.0, 0.0,
            new TrapezoidProfile.Constraints(5, 5));

    public TelescopesIOComp(int leftMotorID, int rightMotorID) {

        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

    }

    public double getLeftPositionMeters() {
        // pls help me D:
        // this is a disaster
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightPositionMeters() {
        return rightMotor.getEncoder().getPosition();
    }

    @Override
    public void updateInputs(TelescopesIOInputs inputs) {
        inputs.leftPositionMeters = getLeftPositionMeters();
        inputs.rightPositionMeters = getRightPositionMeters();
    }

    @Override
    public void resetLeftEncoder() {
        leftMotor.getEncoder().setPosition(0);
    }

    @Override
    public void resetRightEncoder() {
        rightMotor.getEncoder().setPosition(0);
    }

    @Override
    public void resetControllers() {
        leftController.reset(getLeftPositionMeters());
        rightController.reset(getRightPositionMeters());

    }

    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.set(volts/12);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.set(volts/12);
    }

    @Override
    public void setLeftDesiredPositionMeters(double desiredPosition) {
        double output = leftController.calculate(getLeftPositionMeters(), desiredPosition);
        setLeftVoltage(output);
    }

    @Override
    public void setRightDesiredPositionMeters(double desiredPosition) {
        double output = rightController.calculate(getRightPositionMeters(), desiredPosition);
        setRightVoltage(output);
    }

    @Override
    public void setPD(double p, double d) {
        leftController.setP(p);
        leftController.setP(d);

        rightController.setP(p);
        rightController.setP(d);
    }
    
}
