// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.MeasureKs;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.turret.Tracking;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeWheelsIOComp;
import frc.robot.subsystems.intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.rotationarms.*;
import frc.robot.subsystems.shooter.ShooterIOComp;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.telescopes.TelescopesIOComp;
import frc.robot.subsystems.telescopes.TelescopesSubsystem;
import frc.robot.subsystems.tower.TowerIOComp;
import frc.robot.subsystems.tower.TowerSubsystem;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOComp;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.TunableNumber;

public class RobotContainer {
    private final Drive drive;
    //private final IntakeSubsystem intakeSubsystem;
    private final RotationArms rotationArms;
    //private final ShooterSubsystem shooterSubsystem;
    private final TelescopesSubsystem telescopes;
    private final TowerSubsystem towerSubsystem;
    private final Turret turret;

    private final ShooterSubsystem shooterSubsystem;

    private final IntakeWheelsSubsystem intakeWheels;

    private final Vision vision;


    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    private final DriveWithJoysticks driveWithJoysticks;

    public RobotContainer() {
        // Create subsystems
        drive = new Drive(new DriveModuleIO[]{
                new DriveModuleIOComp(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID,
                        CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset),
                new DriveModuleIOComp(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID,
                        CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset),
                new DriveModuleIOComp(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID,
                        CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset),
                new DriveModuleIOComp(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
                        CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset)
        }, new DriveAngleIOComp());

        intakeWheels = new IntakeWheelsSubsystem(new IntakeWheelsIOComp());
        rotationArms = new RotationArms(new RotationArmsIOComp());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOComp());
        telescopes = new TelescopesSubsystem(new TelescopesIOComp());
        towerSubsystem = new TowerSubsystem(new TowerIOComp());
        turret = new Turret(new TurretIOComp());

        vision = new Vision(new VisionIOLimelight());

        // Create commands
        driveWithJoysticks = new DriveWithJoysticks(
                drive,
                () -> -leftStick.getRawAxis(1),
                () -> -leftStick.getRawAxis(0),
                () -> -rightStick.getRawAxis(0)
        );

        // Bind default commands
        drive.setDefaultCommand(driveWithJoysticks);
        turret.setDefaultCommand(new Tracking(vision, turret));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        Command moveUp = new InstantCommand(telescopes::jogUp);
        Command moveDown = new InstantCommand(telescopes::jogDown);

        //Command climbSequence = telescopes.moveToStow.alongWith(rotationArms.moveToStow) // Start moving hooks down and make sure rotation arms are out of the way
        //        .andThen(telescopes.waitForMove) // Wait for the telescopes to finish retracting all the way
        //        .andThen(rotationArms.moveToClimbGrab) // Move the rotation arms into the position above the rung
        //        .andThen(rotationArms.waitForMove) // Wait for the rotation arms to finish moving
        //        .andThen(telescopes.moveToPop) // Move the telescopes up a bit to clear them off the rung
        //        .andThen(telescopes.waitForMove);
                // TODO: swing rotation arms + retract telescopes?

        new JoystickButton(gamepad, Button.kB.value).whileHeld(moveUp);

        new JoystickButton(gamepad, Button.kA.value).whileHeld(moveDown);

        new JoystickButton(gamepad, Button.kY.value)
                .whileHeld(rotationArms.moveToIntake()
                        .alongWith(new Intake(towerSubsystem, intakeWheels)))
                .whenReleased(rotationArms.moveToStow());
        
                
        new JoystickButton(gamepad, Button.kX.value)
                .whenPressed(rotationArms.moveToIntake()
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(-0.5))
                        .alongWith(new InstantCommand(() -> towerSubsystem.setConveyorPercent(-0.5))
                        .alongWith(new InstantCommand(() -> towerSubsystem.setIndexWheelsPercent(-0.5))))))
                .whenReleased(rotationArms.moveToStow()
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(0))
                        .alongWith(new InstantCommand(() -> towerSubsystem.setConveyorPercent(0))
                        .alongWith(new InstantCommand(() -> towerSubsystem.setIndexWheelsPercent(0))))));

        TunableNumber hoodTuningPositionRad = new TunableNumber("hoodTuningPositionRad");
        TunableNumber shooterTuningSpeedRPM = new TunableNumber("shooterTuningSpeedRPM");
        hoodTuningPositionRad.setDefault(0);
        shooterTuningSpeedRPM.setDefault(0);

        new JoystickButton(gamepad, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> shooterSubsystem.setSetpoint(
                        hoodTuningPositionRad.get(),
                        Units.rotationsPerMinuteToRadiansPerSecond(shooterTuningSpeedRPM.get()))))
                .whenReleased(shooterSubsystem::stopShooter);

        new JoystickButton(gamepad, Button.kBack.value)
                .whenHeld(new PrepareToShoot(shooterSubsystem));

        new JoystickButton(gamepad, Button.kRightBumper.value)
                .whenHeld(new InstantCommand(() -> towerSubsystem.setConveyorPercent(1.0))
                        .alongWith(new InstantCommand(() -> towerSubsystem.setIndexWheelsPercent(1.0))))
                .whenReleased(new InstantCommand(() -> towerSubsystem.setConveyorPercent(0))
                        .alongWith(new InstantCommand(() -> towerSubsystem.setIndexWheelsPercent(0))));


        new JoystickButton(rightStick, 2)
            .whenPressed(new InstantCommand(() -> RobotState.getInstance().forceRobotPose(new Pose2d())));
    }

    public Command getAutonomousCommand() {
        return new AutoRoutines(drive);
    }
}