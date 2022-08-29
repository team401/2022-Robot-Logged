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
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.commands.autonomous.AutoRoutines.Paths;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ShootWhileMoving;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.turret.Tracking;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOComp;

public class RobotContainer {

    private final Drive drive;
    private final RotationArms rotationArms;
    private final TelescopesSubsystem telescopes;
    private final Tower tower;
    private final Turret turret;
    private final Shooter shooter;
    private final IntakeWheels intakeWheels;
    private final Vision vision;
    private final IntakeVision intakeVision;
    private final LEDManager ledManager;

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    private final DriveWithJoysticks driveWithJoysticks;

    // Auto trajectories
    private PathPlannerTrajectory[] twoBallPath;
    private PathPlannerTrajectory[] threeBallRightPath;
    private PathPlannerTrajectory[] fiveBallRightPath;
    private PathPlannerTrajectory[] trollLeftPath;
    private PathPlannerTrajectory[] fourBallLeftPath;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
        vision = new Vision(new VisionIOComp());
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
        turret.setDefaultCommand(new Tracking(vision, turret));
        //shooter.setDefaultCommand(new Ramping(shooter));

        configureAutoPaths();

        configureButtonBindings();

        SmartDashboard.putNumber("Shooter Desired", 0);
        SmartDashboard.putNumber("Hood Desired", 0);
    }

    private void configureAutoPaths() {

        // Two Ball
        twoBallPath = new PathPlannerTrajectory[1];
        twoBallPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Two Ball", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, twoBallPath, Paths.TwoBall));

        // Three Ball Right
        threeBallRightPath = new PathPlannerTrajectory[2];
        threeBallRightPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        threeBallRightPath[1] = PathPlanner.loadPath("Right 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Three Ball Right", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, threeBallRightPath, Paths.ThreeBallRight));
        
        // Five Ball Right
        fiveBallRightPath = new PathPlannerTrajectory[4];
        fiveBallRightPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightPath[1] = PathPlanner.loadPath("Right 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightPath[2] = PathPlanner.loadPath("Right 3", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightPath[3] = PathPlanner.loadPath("Right 4", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Five Ball Right", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fiveBallRightPath, Paths.FiveBallRight));
        
        // Troll Left
        trollLeftPath = new PathPlannerTrajectory[2];
        trollLeftPath[0] = PathPlanner.loadPath("Left 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        trollLeftPath[1] = PathPlanner.loadPath("Left 4", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Troll Left", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.TrollLeft));

        // Four Ball Left
        fourBallLeftPath = new PathPlannerTrajectory[3]; 
        fourBallLeftPath[0] = PathPlanner.loadPath("Left 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeftPath[1] = PathPlanner.loadPath("Left 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeftPath[2] = PathPlanner.loadPath("Left 3", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Four Ball Left", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fourBallLeftPath, Paths.FourBallLeft));

        //autoChooser.setDefaultOption("-Five Ball Right-", 
              //new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fiveBallRightPath, Paths.FiveBallRight));
        autoChooser.setDefaultOption("-Troll Left-", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.TrollLeft));
        //autoChooser.setDefaultOption("-Two Ball-", 
                //new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, twoBallPath, Paths.TwoBall));

        // Send path options to driver station
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    private void configureButtonBindings() {

        // photonvision.local:5800 
        // intake cam http://wpilibpi.local:1181/stream.mjpg

        /*CLIMBING BUTTONS*/ 

        // Telescope Up/Down
        /*new POVButton(gamepad, 0)
                .whileHeld(new InstantCommand(() -> telescopes.jogUp()));
        new POVButton(gamepad, 180)
                .whileHeld(new InstantCommand(() -> telescopes.jogDown()));*/

        /*new POVButton(gamepad, 0)
                .whenPressed(new InstantCommand(() -> telescopes.setRightVolts(4))
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(4))))
                .whenReleased(new InstantCommand(() -> telescopes.setRightVolts(0))
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(0))));
        
        new POVButton(gamepad, 180)
                .whenPressed(new InstantCommand(() -> telescopes.setRightVolts(-4))
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(-4))))
                .whenReleased(new InstantCommand(() -> telescopes.setRightVolts(0))
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(0))));*/
        
        // Climb Sequence
        new JoystickButton(gamepad, Button.kX.value)
                .whenPressed(new InstantCommand(() -> ledManager.setClimb(true)))
                .whenHeld(new ClimbSequence(telescopes, rotationArms, gamepad))
                .whenReleased(new InstantCommand(() -> ledManager.setClimb(false)));

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
        
        // Prepare to shoot
        new JoystickButton(gamepad, Button.kRightBumper.value)
                .whenHeld(new PrepareToShoot(shooter, tower));
                        
        // Shoot
        new JoystickButton(gamepad, Button.kY.value)
                .whenHeld(new Shoot(tower, shooter));

        // Shooter RPM Offset (Makes minor adjustments during a game)
        new JoystickButton(leftStick, 3)
                .whenPressed(new InstantCommand(() -> shooter.incrementRPMOffset(-10)));
        new JoystickButton(leftStick, 4)
                .whenPressed(new InstantCommand(() -> shooter.incrementRPMOffset(10)));

        /*OTHERS*/

        // Reset Gyro
        new JoystickButton(rightStick, 2)
                .whenPressed(new InstantCommand(() -> RobotState.getInstance().forceRobotPose(new Pose2d())));

        // Center Turret
        new JoystickButton(gamepad, Button.kA.value)
                .whenPressed(new InstantCommand(() -> turret.setZeroOverride(true)))
                .whenReleased(new InstantCommand(() -> turret.setZeroOverride(false)));
                
        // Rotation Home
        new Trigger(() -> (gamepad.getLeftTriggerAxis() > 0.3))
                .whenActive(new InstantCommand(() -> rotationArms.home(), rotationArms));

        // Telescope Home
        new Trigger(() -> (gamepad.getRightTriggerAxis() > 0.3))
                .whenActive(new InstantCommand(() -> telescopes.home(), telescopes));
        
        // Kill Turret
        new JoystickButton(leftStick, 9)
                .whenPressed(new InstantCommand(() -> turret.kill()));

        // Un Kill Turret
        new JoystickButton(leftStick, 10)
                .whileHeld(new InstantCommand(() -> turret.unkill()));

        // Stop Climb Sequence
        new JoystickButton(leftStick, 8)
                .whenPressed(new InstantCommand(() -> telescopes.stop(), telescopes)
                        .alongWith(new InstantCommand(() -> rotationArms.stop(), rotationArms)));

        // Robot Relative Drive
        new JoystickButton(leftStick, Joystick.ButtonType.kTrigger.value)
                .whenHeld(new DriveWithJoysticks(
                        drive,
                        () -> -leftStick.getRawAxis(1),
                        () -> -leftStick.getRawAxis(0),
                        () -> -rightStick.getRawAxis(0),
                        false
                ));

        // Robot Relative Drive
        new JoystickButton(rightStick, Joystick.ButtonType.kTrigger.value)
                .whenHeld(new ShootWhileMoving(
                        drive,
                        () -> -leftStick.getRawAxis(1),
                        () -> -leftStick.getRawAxis(0),
                        () -> -rightStick.getRawAxis(0),
                        true
                ));

        // Rotation Arm Overrides
        new JoystickButton(rightStick, 7)
                .whenPressed(new InstantCommand(() -> rotationArms.overrideLeftPercent(0.25), rotationArms))
                .whenReleased(new InstantCommand(() -> rotationArms.overrideLeftPercent(0), rotationArms));
        
        new JoystickButton(rightStick, 8)
                .whenPressed(new InstantCommand(() -> rotationArms.overrideLeftPercent(-0.25), rotationArms))
                .whenReleased(new InstantCommand(() -> rotationArms.overrideLeftPercent(0), rotationArms));

        new JoystickButton(rightStick, 6)
                .whenPressed(new InstantCommand(() -> rotationArms.overrideRightPercent(0.25), rotationArms))
                .whenReleased(new InstantCommand(() -> rotationArms.overrideRightPercent(0), rotationArms));

        new JoystickButton(rightStick, 9)
                .whenPressed(new InstantCommand(() -> rotationArms.overrideRightPercent(-0.25), rotationArms))
                .whenReleased(new InstantCommand(() -> rotationArms.overrideRightPercent(0), rotationArms));

        new JoystickButton(rightStick, 10)
                .whenPressed(new InstantCommand(() -> rotationArms.setZero()));

        // Climbing Overrides
        new JoystickButton(leftStick, 7)
                .whenPressed(new InstantCommand(() -> rotationArms.setGoalOverride(true))
                .andThen(new InstantCommand(() -> rotationArms.setGoalOverride(false))));

        new JoystickButton(leftStick, 6)
                .whenPressed(new InstantCommand(() -> telescopes.setGoalOverride(true))
                .andThen(new InstantCommand(() -> telescopes.setGoalOverride(false))));

        // Reset Balls
        /*new JoystickButton(leftStick, 5)
                .whenPressed(new InstantCommand(() -> shooter.setSetpoint(0.27, Units.rotationsPerMinuteToRadiansPerSecond(1000))))
                .whenReleased(new InstantCommand(() -> shooter.stopShooter()));
                //.whenPressed(new InstantCommand(() -> tower.resetBalls()));*/


        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}