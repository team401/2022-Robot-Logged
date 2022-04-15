package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
import frc.robot.commands.drive.QuickTurn;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.turret.ForceSetPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intakevision.IntakeVision;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class AutoRoutines extends SequentialCommandGroup {
    
    public enum Paths {
        TwoBall, 
        ThreeBallRight, FiveBallRight,
        TrollLeft, FourBallLeft
    }

    public AutoRoutines(Drive drive, RotationArms rotationArms, Shooter shooter, Turret turret, Tower tower, IntakeWheels intake, IntakeVision intakeVision, Vision vision, PathPlannerTrajectory[] path, Paths pathPlan) {
        
        SequentialCommandGroup sequentialCommands = new SequentialCommandGroup(
            new InstantCommand(() -> vision.turnOnLeds()),
            rotationArms.moveToIntake(),
            rotationArms.waitForMove(),
            new Intake(tower, intake, rotationArms)
                .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[0], true)
                    .andThen(new WaitCommand(0.25))),
            new InstantCommand(() -> vision.turnOnLeds()),
            new Shoot(tower, shooter).withTimeout(1.5)
        );

        if (pathPlan == Paths.TwoBall) {
            sequentialCommands.addCommands(
                rotationArms.moveToStow()
            );
        }
        
        if (pathPlan == Paths.ThreeBallRight || pathPlan == Paths.FiveBallRight) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)
                        .andThen(new WaitCommand(0.25))),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }
        if (pathPlan == Paths.FiveBallRight) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[2], false)
                        .andThen(new WaitCommand(1))),

                new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[3], false),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }

        if (pathPlan == Paths.FourBallLeft) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)
                        .andThen(new WaitCommand(1))),

                new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[2], false),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }

        if (pathPlan == Paths.FourBallLeft) {
            sequentialCommands.addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)
                        .andThen(new WaitCommand(1))),

                new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[2], false),
                rotationArms.moveToStow(),
                new Shoot(tower, shooter).withTimeout(1.5)
            );
        }

        addCommands(
            new PrepareToShoot(shooter, tower)
                .raceWith(sequentialCommands)
        );

        if (pathPlan == Paths.TrollLeft) {
            addCommands(
                new Intake(tower, intake, rotationArms)
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)),
                rotationArms.moveToStow(),
                new InstantCommand(() -> turret.setZeroOverride(true)),
                new InstantCommand(() -> shooter.setSetpoint(0.63, Units.rotationsPerMinuteToRadiansPerSecond(1500))),
                new WaitCommand(1),
                new Shoot(tower, shooter).withTimeout(3),
                new InstantCommand(() -> turret.setZeroOverride(false)),
                new InstantCommand(() -> shooter.stopShooter())
            );
        }
        
    }

}