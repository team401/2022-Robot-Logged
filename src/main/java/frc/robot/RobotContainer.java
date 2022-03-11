// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.rotationarms.*;
import frc.robot.subsystems.telescopes.TelescopesIOComp;
import frc.robot.subsystems.telescopes.TelescopesSubsystem;

public class RobotContainer {
    private final Drive drive;
    //private final IntakeSubsystem intakeSubsystem;
    private final RotationArms rotationArms;
    //private final ShooterSubsystem shooterSubsystem;
    private final TelescopesSubsystem telescopes;
    //private final TowerSubsystem towerSubsystem;
    //private final TurretSubsystem turretSubsystem;


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

        //intakeSubsystem = new IntakeSubsystem(new IntakeWheelsIOComp());
        rotationArms = new RotationArms(new RotationArmsIOComp());
        //shooterSubsystem = new ShooterSubsystem(new ShooterIOComp());
        telescopes = new TelescopesSubsystem(new TelescopesIOComp());
        //towerSubsystem = new TowerSubsystem(new TowerIOComp());
        //turretSubsystem = new TurretSubsystem(new TurretIOComp());

        // Create commands
        driveWithJoysticks = new DriveWithJoysticks(
                drive,
                () -> -leftStick.getRawAxis(1),
                () -> -leftStick.getRawAxis(0),
                () -> -rightStick.getRawAxis(0)
        );

        // Bind default commands
        drive.setDefaultCommand(driveWithJoysticks);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        Command moveUp = new InstantCommand(telescopes::jogUp);
        Command moveDown = new InstantCommand(telescopes::jogDown);

        Command climbSequence = telescopes.moveToStow.alongWith(rotationArms.moveToStow) // Start moving hooks down and make sure rotation arms are out of the way
                .andThen(telescopes.waitForMove) // Wait for the telescopes to finish retracting all the way
                .andThen(rotationArms.moveToClimbGrab) // Move the rotation arms into the position above the rung
                .andThen(rotationArms.waitForMove) // Wait for the rotation arms to finish moving
                .andThen(telescopes.moveToPop) // Move the telescopes up a bit to clear them off the rung
                .andThen(telescopes.waitForMove);
                // TODO: swing rotation arms + retract telescopes?

        JoystickButton b = new JoystickButton(gamepad, Button.kB.value);
        b.whileHeld(moveUp);
        JoystickButton a = new JoystickButton(gamepad, Button.kA.value);
        a.whileHeld(moveDown);
        JoystickButton x = new JoystickButton(gamepad, Button.kX.value);
        x.whenHeld(climbSequence);
    }

    public Command getAutonomousCommand() {
        SwerveModuleState zero = new SwerveModuleState();
        SwerveModuleState[] zeros = new SwerveModuleState[]{zero, zero, zero, zero};
        return new InstantCommand(() -> drive.setGoalModuleStates(zeros), drive).andThen(new WaitCommand(2.0))
                .andThen(new InstantCommand(() -> drive.setDriveVoltages(new double[]{4, 4, 4, 4}), drive)).andThen(new WaitCommand(10.0));
    }
}