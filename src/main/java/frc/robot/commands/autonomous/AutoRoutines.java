package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;

public class AutoRoutines extends SequentialCommandGroup {
    
    public AutoRoutines(Drive drive, RotationArms rotationArms, Shooter shoot, Tower tower, IntakeWheels intakeWheels, Vision vision) {
        
        addCommands(

            /*
            old code that actually works:
            new PrepareToShoot(shoot)
                .raceWith(new WaitCommand(1.5)
                .andThen(new InstantCommand(() -> tower.setConveyorPercent(1.0))
                    .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(1.0))))
                .andThen(new WaitCommand(2))),
            new InstantCommand(() -> tower.setConveyorPercent(0.0))
                .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(0.0))),
            new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), PathPlanner.loadPath("PID Test Path", 4, 5))
            */

            new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), PathPlanner.loadPath("Right Tarmac Path", 2, 1.5))

            /*new InstantCommand(() -> rotationArms.moveToIntake()),
            new ParallelCommandGroup(
                new AutoShoot(shoot, tower, vision),
                new Intake(tower, intakeWheels)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), PathPlanner.loadPath("Right Tarmac Path", 3, 0.5)))
                    .andThen(new InstantCommand(() -> rotationArms.moveToStow()))

            )*/
        ); 

    }

}
