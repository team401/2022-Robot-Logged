package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TelescopesIOComp implements TelescopesIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final ProfiledPIDController leftController = new ProfiledPIDController(1.5, 0.0, 0.0,
            new TrapezoidProfile.Constraints(5, 5));
    private final ProfiledPIDController rightController = new ProfiledPIDController(1.5, 0.0, 0.0,
            new TrapezoidProfile.Constraints(5, 5));

    public TelescopesIOComp(int leftMotorID, int rightMotorID) {

        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftEncoder.setPositionConversionFactor(1.0/4096*2*Math.PI);
        rightEncoder.setPositionConversionFactor(1.0/4096*2*Math.PI);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

    }

    public double getLeftPositionRad() {
        return leftEncoder.getPosition();
    }

    public double getRightPositionRad() {
        return rightEncoder.getPosition();
    }

    @Override
    public void updateInputs(TelescopesIOInputs inputs) {
        inputs.leftPositionRad = getLeftPositionRad();
        inputs.rightPositionRad = getRightPositionRad();
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
        leftController.reset(getLeftPositionRad());
        rightController.reset(getRightPositionRad());

    }

    @Override
    public void setLeftPercent(double percent) {
        leftMotor.set(percent);
    }

    @Override
    public void setRightPercent(double percent) {
        rightMotor.set(percent);
    }

    @Override
    public void setLeftDesiredPositionRad(double desiredPosition) {
        double output = leftController.calculate(getLeftPositionRad(), desiredPosition);
        setLeftPercent(output);
    }

    @Override
    public void setRightDesiredPositionRad(double desiredPosition) {
        double output = rightController.calculate(getRightPositionRad(), desiredPosition);
        setRightPercent(output);
    }

    @Override
    public void setPD(double p, double d) {
        leftController.setP(p);
        leftController.setP(d);

        rightController.setP(p);
        rightController.setP(d);
    }
    
}
