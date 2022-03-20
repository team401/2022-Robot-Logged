package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;

public class PathPlannerTrajectoryCommand extends CommandBase {
    private final Drive drive;
    private Pose2d latestFieldToVehicle;
    private final RobotState robotState;
    private final PathPlannerTrajectory trajectory;
    private final PIDController xController = new PIDController(DriveConstants.followTrajectoryXControllerKp.get(), 0, DriveConstants.followTrajectoryXControllerKd.get());
    private final PIDController yController = new PIDController(DriveConstants.followTrajectoryYControllerKp.get(), 0, DriveConstants.followTrajectoryYControllerKd.get());
    private final ProfiledPIDController thetaController = 
        new ProfiledPIDController(
            DriveConstants.followTrajectoryThetaControllerKp.get(), 
            0, 
            DriveConstants.followTrajectoryThetaControllerKd.get(), 
            new TrapezoidProfile.Constraints(0, 0)
        );

    private final HolonomicDriveController controller;

    //private final PPSwerveControllerCommand trajectoryController;
    private final PathPlannerState pathState;

    private final Turret turret;
    
    private final Timer timer = new Timer();

    public PathPlannerTrajectoryCommand(Drive drive, RobotState robotState, Turret turret,  PathPlannerTrajectory trajectory) {
        
        this.drive = drive;
        this.robotState = robotState;
        this.trajectory = trajectory;
        this.turret = turret;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller  = new HolonomicDriveController(xController, yController, thetaController);

        pathState = trajectory.getInitialState();

        /*trajectoryController = 
            new PPSwerveControllerCommand(
                trajectory,
                drive::getPose,
                DriveConstants.kinematics,
                xController,
                yController,
                thetaController,
                drive::setGoalModuleStates,
                drive
            );*/

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        robotState.forceRobotPose(trajectory.getInitialState().poseMeters);
        //drive.resetOdometry(new Pose2d(pathState.poseMeters.getTranslation(), pathState.holonomicRotation));

    }

    @Override
    public void execute() {
        
        if (DriveConstants.followTrajectoryXControllerKp.hasChanged())
            xController.setP(DriveConstants.followTrajectoryXControllerKp.get());
        if (DriveConstants.followTrajectoryXControllerKd.hasChanged())
            xController.setD(DriveConstants.followTrajectoryXControllerKd.get());

        if (DriveConstants.followTrajectoryYControllerKp.hasChanged())
            yController.setP(DriveConstants.followTrajectoryYControllerKp.get());
        if (DriveConstants.followTrajectoryYControllerKd.hasChanged())
            yController.setD(DriveConstants.followTrajectoryYControllerKd.get());
        
        if (DriveConstants.followTrajectoryThetaControllerKp.hasChanged())
            thetaController.setP(DriveConstants.followTrajectoryThetaControllerKp.get());
        if (DriveConstants.followTrajectoryThetaControllerKd.hasChanged())
            thetaController.setD(DriveConstants.followTrajectoryThetaControllerKd.get());

        latestFieldToVehicle = robotState.getLatestFieldToVehicle();
        
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

        Logger.getInstance().recordOutput("Auto/desiredStateX", trajectory.sample(timer.get()).poseMeters.getX());
        Logger.getInstance().recordOutput("Auto/desiredStateY", trajectory.sample(timer.get()).poseMeters.getY());
        Logger.getInstance().recordOutput("Auto/desiredStateRotation", trajectory.sample(timer.get()).poseMeters.getRotation().getDegrees());
    
        Logger.getInstance().recordOutput("Auto/actualX", drive.getPose().getX());
        Logger.getInstance().recordOutput("Auto/actualY", drive.getPose().getY());
        Logger.getInstance().recordOutput("Auto/actualRotation", drive.getPose().getRotation().getDegrees());


        ChassisSpeeds adjustedSpeeds = controller.calculate(
            latestFieldToVehicle, desiredState, desiredState.holonomicRotation);

        drive.setGoalChassisSpeeds(adjustedSpeeds);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

}