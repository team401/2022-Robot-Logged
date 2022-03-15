package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines extends SequentialCommandGroup {
    
    public AutoRoutines(Drive drive) {

        addCommands(
            new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), PathPlanner.loadPath("PID Test Path", 2, 2))
        ); 
    }

}
