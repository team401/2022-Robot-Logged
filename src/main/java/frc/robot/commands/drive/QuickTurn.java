package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.DriveWithJoysticks.AxisProcessor;

public class QuickTurn extends CommandBase {
    
    private final Drive drive;
    private final double desiredAngleRad;

    private final PIDController controller = new PIDController(5, 0, 0);

    public QuickTurn(Drive drive, double desiredAngleRad) {
        this.drive = drive;
        this.desiredAngleRad = desiredAngleRad;

        addRequirements(drive);
    }

    @Override
    public void execute() {

      double output = controller.calculate(drive.getPose().getRotation().getRadians(), desiredAngleRad);
      drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, output));

    }

    @Override
    public void end(boolean isInterrupted) {

        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));

    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getPose().getRotation().getRadians() - desiredAngleRad) < Units.degreesToRadians(3);
    }

}
