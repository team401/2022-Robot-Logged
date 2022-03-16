package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;

public class AutoRoutines extends SequentialCommandGroup {
    
    public AutoRoutines(Drive drive, Shooter shoot, Tower tower) {

        addCommands(
            new PrepareToShoot(shoot)
                .raceWith(new WaitCommand(5)
                .andThen(new InstantCommand(() -> tower.setConveyorPercent(1.0))
                    .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(1.0))))
                .andThen(new WaitCommand(3))),
            //new WaitCommand(2),
            new InstantCommand(() -> tower.setConveyorPercent(0.0))
                .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(0.0))),
            new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), PathPlanner.loadPath("PID Test Path", 2, 2))
        ); 
    }

}
