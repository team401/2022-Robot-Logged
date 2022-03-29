package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.PrepareToShoot;
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
        TwoBall, 
        ThreeBallRight, FiveBallRight,
        TrollLeft, FourBallLeft
    }

    public AutoRoutines(Drive drive, RotationArms rotationArms, Shooter shooter, Turret turret, Tower tower, IntakeWheels intake, Vision vision, PathPlannerTrajectory[] path, Paths pathPlan) {
        
        SequentialCommandGroup sequentialCommands = new SequentialCommandGroup(
            rotationArms.moveToIntake(),
            rotationArms.waitForMove(),
            new Intake(tower, intake, rotationArms)
                .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[0], true)),
            new ForceSetPosition(turret, vision, new Rotation2d()).withTimeout(0.5),
            new Shoot(tower, shooter).withTimeout(1.5)
        );
        
        if (pathPlan == Paths.ThreeBallRight || pathPlan == Paths.FiveBallRight) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], false)),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }
        if (pathPlan == Paths.FiveBallRight) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], false)
                        .andThen(new WaitCommand(2))),

                new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[3], false),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }

        if (pathPlan == Paths.FourBallLeft) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], false)
                        .andThen(new WaitCommand(2))),

                new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], false),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }

        if (pathPlan == Paths.FourBallLeft) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], false)
                        .andThen(new WaitCommand(2))),

                new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[2], false),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }

        if (pathPlan == Paths.TrollLeft) {
            addCommands(
                new SequentialCommandGroup(
                    new PrepareToShoot(shooter)
                        .raceWith(sequentialCommands),
                    new InstantCommand(() -> shooter.setSetpoint(1, 1000), shooter),
                    new Intake(tower, intake, rotationArms)
                        .raceWith(new PathPlannerTrajectoryCommand(drive, RobotState.getInstance(), turret, path[1], false)),
                    rotationArms.moveToStow(),
                    new WaitCommand(1),
                    new Shoot(tower, shooter).withTimeout(5),
                    new InstantCommand(() -> shooter.stopShooter(), shooter)
                );
            );
        }
        else {
            addCommands(
                new PrepareToShoot(shooter),
                sequentialCommands
            );
        }
    }

}
