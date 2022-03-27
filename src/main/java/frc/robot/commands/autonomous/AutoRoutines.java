package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class AutoRoutines extends ParallelCommandGroup {
    
    public enum Paths {
        TwoBall, 
        ThreeBallRight, FiveBallRight, SixBallRight,
        FourBallLeft, FiveBallLeft, SixBallLeft
    }

    public AutoRoutines(Drive drive, RotationArms rotationArms, Shooter shooter, Turret turret, Tower tower, IntakeWheels intake, Vision vision, PathPlannerTrajectory[] path, Paths pathPlan) {
        
        SequentialCommandGroup sequentialCommands = new SequentialCommandGroup(
            rotationArms.moveToIntake(),
            rotationArms.waitForMove(),
            new Intake(tower, intake, rotationArms)
                .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0], true)),
            rotationArms.moveToStow(),
            new Shoot(tower, shooter).withTimeout(2)
        );
        
        if (pathPlan == Paths.ThreeBallRight || pathPlan == Paths.FiveBallRight || pathPlan == Paths.SixBallRight) {
            sequentialCommands.addCommands(
                rotationArms.moveToIntake(),
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], true)),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(2)
            );
        }
        if (pathPlan == Paths.FiveBallRight || pathPlan == Paths.SixBallRight) {
            sequentialCommands.addCommands(
                rotationArms.moveToIntake(),
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], true)
                        .andThen(new WaitCommand(3))),
                rotationArms.moveToStow(),

                new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[3], true),
                new Shoot(tower, shooter).withTimeout(2)
            );
        }
        if (pathPlan == Paths.SixBallRight) {
            sequentialCommands.addCommands(
                rotationArms.moveToIntake(),
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[4], true)),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(2)
            );
        }

        if (pathPlan == Paths.FourBallLeft || pathPlan == Paths.FiveBallLeft || pathPlan == Paths.SixBallLeft) {
            sequentialCommands.addCommands(
                rotationArms.moveToIntake(),
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], true)
                        .andThen(new WaitCommand(3))),
                rotationArms.moveToStow(),

                new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], true),
                new Shoot(tower, shooter).withTimeout(2)
            );
        }
        if (pathPlan == Paths.FiveBallLeft || pathPlan == Paths.SixBallLeft) {
            sequentialCommands.addCommands(
                rotationArms.moveToIntake(),
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[4], true)),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(2)
            );
        }

        addCommands(
            new PrepareToShoot(shooter),
            sequentialCommands
        );

        /*switch (pathPlan) {

            case TwoBall:
                addCommands(
                    new PrepareToShoot(shooter),
                    new SequentialCommandGroup(
                        rotationArms.moveToIntake(),
                        rotationArms.waitForMove(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2)
                    )
                );
                break;
            
            case ThreeBallRight:
                addCommands(
                    new PrepareToShoot(shooter),
                    new SequentialCommandGroup(
                        rotationArms.moveToIntake(),
                        rotationArms.waitForMove(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2)
                    )
                );
                break;

            case FiveBallRight:
                addCommands(
                    new PrepareToShoot(shooter),
                    new SequentialCommandGroup(
                        rotationArms.moveToIntake(),
                        rotationArms.waitForMove(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], true)
                                .andThen(new WaitCommand(3))),
                        rotationArms.moveToStow(),

                        new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[3], true),
                        new Shoot(tower, shooter).withTimeout(2)
                    )
                );
                break;

            case SixBallRight:
                addCommands(
                    new PrepareToShoot(shooter),
                    new SequentialCommandGroup(
                        rotationArms.moveToIntake(),
                        rotationArms.waitForMove(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], true)
                                .andThen(new WaitCommand(3))),
                        rotationArms.moveToStow(),

                        new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[3], true),
                        new Shoot(tower, shooter).withTimeout(2),

                        rotationArms.moveToIntake(),
                        new Intake(tower, intake, rotationArms)
                            .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[4], true)),
                        rotationArms.moveToStow(),
                        new Shoot(tower, shooter).withTimeout(2)
                    )
                );
                break;

        }*/

    }

}
