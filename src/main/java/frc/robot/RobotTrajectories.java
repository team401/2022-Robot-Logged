package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class RobotTrajectories {

    /**
     * Defines the trajectories to be run through auto
     * 
     */

    // configures the maximum velocity and accel for the trajectories

    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.kMaxVelocityMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        .setKinematics(DriveConstants.kinematics);

}