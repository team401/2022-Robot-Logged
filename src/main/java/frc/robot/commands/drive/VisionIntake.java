package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.drive.DriveWithJoysticks.AxisProcessor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakevision.IntakeVision;

public class VisionIntake extends CommandBase {

    private final IntakeVision intakeVision;
    private final Drive drive;

    private final DoubleSupplier yPercent;
    private final AxisProcessor yProcessor = new AxisProcessor(false);

    private final PIDController controller = new PIDController(DriveConstants.intakeVisionKP.get(), 0, DriveConstants.intakeVisionKD.get());

    
    public VisionIntake(IntakeVision intakeVision, Drive drive, DoubleSupplier yPercent) {

        this.intakeVision = intakeVision;
        this.drive = drive;

        this.yPercent = yPercent;

        addRequirements(intakeVision, drive);

    }

    @Override
    public void initialize(){
        yProcessor.reset(yPercent.getAsDouble());
    }

    @Override
    public void execute() {

        if (DriveConstants.intakeVisionKP.hasChanged())
            controller.setP(DriveConstants.intakeVisionKP.get());
        if (DriveConstants.intakeVisionKD.hasChanged())
            controller.setD(DriveConstants.intakeVisionKD.get());

        if (intakeVision.hasTarget()) {
            double omegaOutput = controller.calculate(-intakeVision.getTX(), 0);
            double yOutput = yProcessor.processJoystickInputs(yPercent.getAsDouble()) * DriveConstants.maxSpeedMPerS;
            ChassisSpeeds targetSpeeds = new ChassisSpeeds(0, yOutput, omegaOutput);

            drive.setGoalChassisSpeeds(targetSpeeds);

        }

    }
    
}
