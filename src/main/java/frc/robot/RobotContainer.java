// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.turret.ManualControl;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeWheelsIOComp;
import frc.robot.subsystems.intakevision.IntakeVision;
import frc.robot.subsystems.ledManager.LEDManager;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.rotationarms.*;
import frc.robot.subsystems.shooter.ShooterIOComp;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.telescopes.TelescopesIOComp;
import frc.robot.subsystems.telescopes.TelescopesSubsystem;
import frc.robot.subsystems.tower.TowerIOComp;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOComp;

public class RobotContainer {

    private final Drive drive;
    private final RotationArms rotationArms;
    private final TelescopesSubsystem telescopes;
    private final Tower tower;
    private final Turret turret;
    private final Shooter shooter;
    private final IntakeWheels intakeWheels;
    private final IntakeVision intakeVision;
    private final LEDManager ledManager;

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    private final DriveWithJoysticks driveWithJoysticks;

    public RobotContainer() {

        // Create subsystems; each subsystem takes its respective IOComp as a parameter

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

        intakeWheels = new IntakeWheels(new IntakeWheelsIOComp());
        rotationArms = new RotationArms(new RotationArmsIOComp());
        shooter = new Shooter(new ShooterIOComp());
        telescopes = new TelescopesSubsystem(new TelescopesIOComp());
        tower = new Tower(new TowerIOComp());
        turret = new Turret(new TurretIOComp());
        intakeVision = new IntakeVision();
        ledManager = new LEDManager();

        // Create commands  
        driveWithJoysticks = new DriveWithJoysticks(
                drive,
                () -> -leftStick.getRawAxis(1),
                () -> -leftStick.getRawAxis(0),
                () -> -rightStick.getRawAxis(0),
                true
        );

        // set default commands
        drive.setDefaultCommand(driveWithJoysticks);
        turret.setDefaultCommand(new ManualControl(turret, () -> gamepad.getRightX(), () -> gamepad.getRightY()));
        shooter.setDefaultCommand(new PrepareToShoot(shooter, () -> gamepad.getRightTriggerAxis()));

        configureButtonBindings();

        SmartDashboard.putNumber("Shooter Desired", 0);
        SmartDashboard.putNumber("Hood Desired", 0);
    }

    private void configureButtonBindings() {

        // photonvision.local:5800 
        // intake cam http://wpilibpi.local:1181/stream.mjpg

        /*CLIMBING BUTTONS*/ 

        // Telescope Up/Down
        new POVButton(gamepad, 0)
                .whileHeld(new InstantCommand(() -> telescopes.jogUp()));
        new POVButton(gamepad, 180)
                .whileHeld(new InstantCommand(() -> telescopes.jogDown()));

        /*INTAKE BUTTONS*/ 

        // Rotation Arms Intake/Stow (Without running ball tower)
        new POVButton(gamepad, 90)
                .whenPressed(rotationArms.moveToIntake());
        new POVButton(gamepad, 270)
                .whenPressed(rotationArms.moveToStow());
                
        // Intake
        new JoystickButton(gamepad, Button.kB.value)
                .whenPressed(rotationArms.moveToIntake())
                .whenHeld(new Intake(tower, intakeWheels, rotationArms))
                .whenReleased(rotationArms.moveToStow());
        
        // Reverse Intake
        new JoystickButton(gamepad, Button.kBack.value)
                .whenPressed(rotationArms.moveToIntake()
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(-0.5))
                        .alongWith(new InstantCommand(() -> tower.setConveyorPercent(-0.5))
                        .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(-0.5))))))
                .whenReleased(rotationArms.moveToStow()
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(0))
                        .alongWith(new InstantCommand(() -> tower.setConveyorPercent(0))
                        .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(0))))));
        
        /*SHOOTING BUTTONS*/ 
                        
        // Shoot
        new JoystickButton(gamepad, Button.kY.value)
                .whenHeld(new Shoot(tower, shooter));

        /*OTHERS*/

        // Reset Gyro
        new JoystickButton(rightStick, 2)
                .whenPressed(new InstantCommand(() -> RobotState.getInstance().forceRobotPose(new Pose2d())));
                
        // Home
        new JoystickButton(gamepad, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> rotationArms.home(), rotationArms))
                .whenPressed(new InstantCommand(() -> telescopes.home(), telescopes));
                
        
    }

    public Command getAutonomousCommand() {
        return null;
    }

}