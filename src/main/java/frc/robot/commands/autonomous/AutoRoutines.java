package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.shooter.SegmentedAutoShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.turret.ForceSetPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class AutoRoutines extends ParallelCommandGroup {
    
    public enum Paths {
        Left, Right, Back
    }

    public AutoRoutines(Drive drive, RotationArms rotationArms, Shooter shooter, Turret turret, Tower tower, IntakeWheels intake, Vision vision, PathPlannerTrajectory[] path, Paths startPosition) {
        
        switch (startPosition) {

            case Left:
            case Right:
                addCommands(
                    new SequentialCommandGroup(
                        new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0]),
                        new WaitCommand(2),
                        new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1]),
                        new WaitCommand(2),
                        new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2])
                    )

                );
                /*addCommands(
                    new PrepareToShoot(shooter),
                    new SequentialCommandGroup(
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0])),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1])),
                        rotationArms.moveToStow(),

                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2])),
                        new Shoot(tower, shooter).withTimeout(3)
                    )

                );*/
                break;

            case Back:
                addCommands(
                    new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0])
                    /*new PrepareToShoot(shooter),
                    new SequentialCommandGroup(
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0])),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2)
                    )*/
                );
                break;

        }

    }

}
