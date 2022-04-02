package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.DriveWithJoysticks.AxisProcessor;

public class QuickTurnWithJoysticks extends CommandBase {
    
    private final Drive drive;

    private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private final PIDController controller = new PIDController(0, 0, 0);

    private final DoubleSupplier leftX;
    private final DoubleSupplier leftY;

    private final AxisProcessor xProcessor = new AxisProcessor(false);
    private final AxisProcessor yProcessor = new AxisProcessor(false);

    public QuickTurnWithJoysticks(Drive drive, DoubleSupplier leftX, DoubleSupplier leftY) {
        this.drive = drive;

        this.leftX = leftX;
        this.leftY = leftY;

        addRequirements(drive);
    }

    @Override
    public void execute() {

        Rotation2d vehicleToGoal = RobotState.getInstance().getVehicleToGoal();
        double filteredAngle = filter.calculate(vehicleToGoal.getRadians());
        

        double xMPerS = xProcessor.processJoystickInputs(leftX.getAsDouble()) * DriveConstants.maxSpeedMPerS;
        double yMPerS = yProcessor.processJoystickInputs(leftY.getAsDouble()) * DriveConstants.maxSpeedMPerS;
        double omegaRadPerS = controller.calculate(drive.getPose().getRotation().getRadians(), filteredAngle);
    
        //Convert to field relative speeds
        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, RobotState.getInstance().getLatestFieldToVehicle().getRotation());
        //ChassisSpeeds targetSpeeds = new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);
    
        drive.setGoalChassisSpeeds(targetSpeeds);

    }

    @Override
    public void end(boolean isInterrupted) {

    }

}
