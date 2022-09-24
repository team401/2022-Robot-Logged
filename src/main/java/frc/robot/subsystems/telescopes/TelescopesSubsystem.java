package frc.robot.subsystems.telescopes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.telescopes.TelescopesIO.TelescopesIOInputs;

public class TelescopesSubsystem extends SubsystemBase {
    private final TelescopesIO io;
    private final TelescopesIOInputs ioInputs = new TelescopesIOInputs();

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.telescopeArmKp, 0, ClimberConstants.telescopeArmKd,
        new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM)
    );

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.telescopeArmKp, 0, ClimberConstants.telescopeArmKd,
        new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM)
    );

    private final double deltaTime = 0.02; // 20ms between each cycle

    private double desiredPositionM = 0; // position the telescopes attempt to reach in periodic

    private boolean motorOverride = false; // true = periodic doesn't set motor voltage/percent
    private boolean atGoalOverride = false; // true = atGoal() always returns true

    private boolean leftHomed = false;
    private boolean rightHomed = false;

    private final Timer leftHomeTimer = new Timer();
    private final Timer rightHomeTimer = new Timer();

    public TelescopesSubsystem(TelescopesIO io) {
        this.io = io;

        leftController.setGoal(ClimberConstants.telescopeHomePositionM);
        rightController.setGoal(ClimberConstants.telescopeHomePositionM);

        leftController.setTolerance(ClimberConstants.telescopeGoalToleranceM);
        rightController.setTolerance(ClimberConstants.telescopeGoalToleranceM);

        leftHomeTimer.start();
        rightHomeTimer.start();
    }
    
    @Override
    public void periodic() {
        long m_Start = System.currentTimeMillis();

        if (!DriverStation.isEnabled()) {
            if (io.isLeftLidarValid())
                leftController.reset(ioInputs.leftLidarPositionM);
            else
                leftController.reset(ioInputs.leftEncoderPositionM);
            if (io.isRightLidarValid())
                rightController.reset(ioInputs.rightLidarPositionM);
            else
                rightController.reset(ioInputs.rightEncoderPositionM);
            
            leftHomeTimer.reset();
            rightHomeTimer.reset();
        }

        if (DriverStation.isEnabled() && !motorOverride) {
            // Left
            if (io.isLeftLidarValid()) {
                double output = leftController.calculate(ioInputs.leftLidarPositionM, desiredPositionM);
                output -= output < 0 ? 0.7 : 0; // FF
                io.setLeftVolts(output);

                leftHomed = true;
                if (Math.abs(ioInputs.leftLidarPositionM - ioInputs.leftEncoderPositionM) > ClimberConstants.telescopeGoalToleranceM)
                    io.setLeftEncoder(ioInputs.leftLidarPositionM);
            }
            else {
                if (leftHomed) {
                    double output = leftController.calculate(ioInputs.leftEncoderPositionM, desiredPositionM);
                    output -= output < 0 ? 0.7 : 0; // FF
                    io.setLeftVolts(output);
                }
                else {
                    if (Math.abs(ioInputs.leftVelocityMPerS) > ClimberConstants.telescopeHomingThresholdMPerS) {
                        leftHomeTimer.reset();
                    }
                    else if (leftHomeTimer.hasElapsed(ClimberConstants.homingTimeS)){
                        leftHomed = true;
                        io.setLeftEncoder(ioInputs.leftEncoderPositionM);
                        io.setLeftVolts(0);
                        leftController.reset(ioInputs.leftEncoderPositionM);
                    }
                    else {
                        io.setLeftVolts(ClimberConstants.telescopeHomingVolts);
                    }
                }
            }

            // Right
            if (io.isRightLidarValid()) {
                double output = rightController.calculate(ioInputs.rightLidarPositionM, desiredPositionM);
                output -= output < 0 ? 0.7 : 0; // FF
                io.setRightVolts(output);

                rightHomed = true;
                if (Math.abs(ioInputs.rightLidarPositionM - ioInputs.rightEncoderPositionM) > ClimberConstants.telescopeGoalToleranceM)
                    io.setRightEncoder(ioInputs.rightLidarPositionM);
            }
            else {
                if (rightHomed) {
                    double output = rightController.calculate(ioInputs.rightEncoderPositionM, desiredPositionM);
                    output -= output < 0 ? 0.7 : 0; // FF
                    io.setRightVolts(output);
                }
                else {
                    if (Math.abs(ioInputs.rightVelocityMPerS) > ClimberConstants.telescopeHomingThresholdMPerS) {
                        rightHomeTimer.reset();
                    }
                    else if (rightHomeTimer.hasElapsed(ClimberConstants.homingTimeS)){
                        rightHomed = true;
                        io.setRightEncoder(ioInputs.rightEncoderPositionM);
                        io.setRightVolts(0);
                        rightController.reset(ioInputs.rightEncoderPositionM);
                    }
                    else {
                        io.setRightVolts(ClimberConstants.telescopeHomingVolts);
                    }
                }
            }
        }

        Logger.getInstance().recordOutput("Telescopes/LeftLidarM", ioInputs.leftLidarPositionM);
        Logger.getInstance().recordOutput("Telescopes/RightLidarM", ioInputs.rightLidarPositionM);
        Logger.getInstance().recordOutput("Telescopes/LeftEncoderM", ioInputs.leftEncoderPositionM);
        Logger.getInstance().recordOutput("Telescopes/RightEncoderM", ioInputs.rightEncoderPositionM);
        Logger.getInstance().recordOutput("Telescopes/DesiredPositionM", desiredPositionM);
        Logger.getInstance().recordOutput("Telescopes/LeftControllerSetpointM", leftController.getSetpoint().position);
        Logger.getInstance().recordOutput("Telescopes/RightControllerSetpointM", rightController.getSetpoint().position);
        Logger.getInstance().recordOutput("Telescopes/AtGoal", atGoal());
        Logger.getInstance().recordOutput("Telescopes/AtGoalOverride", atGoalOverride);
        Logger.getInstance().recordOutput("Telescopes/MotorOverride", motorOverride);
        SmartDashboard.putBoolean("Telescopes At Goal", atGoal());

        Logger.getInstance().recordOutput("ExecutionTime/Telescopes", (int)(System.currentTimeMillis() - m_Start));
    }

    public void setLeftPercent(double percent) {
        io.setLeftVolts(percent * 12);
    }

    public void setRightPercent(double percent) {
        io.setRightVolts(percent * 12);
    }

    public void setDesiredPosition(double positionM) {
        desiredPositionM = positionM;
    }

    public void jogUp() {
        desiredPositionM += deltaTime * ClimberConstants.telescopeCruiseVelocityM;
        if (desiredPositionM > ClimberConstants.telescopeMaxPositionM)
            desiredPositionM = ClimberConstants.telescopeMaxPositionM;
    }

    public void jogDown() {
        desiredPositionM -= deltaTime * ClimberConstants.telescopeCruiseVelocityM;
        if (desiredPositionM < ClimberConstants.telescopeHomePositionM)
            desiredPositionM = ClimberConstants.telescopeHomePositionM;
    }

    public boolean atGoal() {
        return atGoalOverride || (leftController.atGoal() && rightController.atGoal());
    }

    public boolean passedRotationSafePosition() {
        double leftPositionM = io.isLeftLidarValid() ? ioInputs.leftLidarPositionM : ioInputs.leftEncoderPositionM;
        double rightPositionM = io.isRightLidarValid() ? ioInputs.rightLidarPositionM : ioInputs.rightEncoderPositionM;
        return leftPositionM <= ClimberConstants.telescopeRotationSafePositionM &&
                rightPositionM <= ClimberConstants.telescopeRotationSafePositionM;
    }

    public void setLeftVolts(double volts) {
        io.setLeftVolts(volts);
    }

    public void setRightVolts(double volts) {
        io.setRightVolts(volts);
    }

    public void stop() {
        setDesiredPosition(io.isLeftLidarValid() ? ioInputs.leftLidarPositionM : ioInputs.leftEncoderPositionM);
    }

    public void setMotorOverride(boolean override) {
        motorOverride = override;
        setLeftVolts(0);
        setRightVolts(0);
    }

    public void setAtGoalOverride(boolean override) {
        atGoalOverride = override;
    }

    // Commands
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
    public final Command waitForRotationSafePosition() { return new WaitUntilCommand(this::passedRotationSafePosition); }
    public final Command moveToPop() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePopAboveRungM), this); }
    public final Command moveToFull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeMaxPositionM), this); }
    public final Command moveToLatch() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeLatchM), this); }
    public final Command moveToPull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePullPositionM), this); }
    public final Command moveToSwing() {return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeSwingPositionM), this); }
}
