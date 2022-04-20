package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.telescopes.TelescopesSubsystem;

public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(TelescopesSubsystem telescopes, RotationArms rotationArms, XboxController gamepad) {

        addRequirements(telescopes, rotationArms);

        addCommands(            
            // TO MID BAR
            // Pull up to mid bar
            telescopes.moveToPull(),
            rotationArms.moveToStow(),
            rotationArms.waitForMove(),
            telescopes.waitForMove(),
            // Rotation arms above mid bar
            rotationArms.moveToClimbGrab(),
            rotationArms.waitForMove(),
            // Telescopes up a bit to clear them off the high bar
            telescopes.moveToPop(),
            telescopes.waitForMove(),


            // TO HIGH BAR
            // Telescopes extend while rotation arms move back to catch high bar
            rotationArms.moveToClimbSwing(),
            //telescopes.moveToSwing(),
            rotationArms.waitForMove(),
            // Telescopes up to above high bar
            telescopes.moveToFull(),
            telescopes.waitForMove(),
            // Rotation arms to contact the high bar with telescopes
            rotationArms.latchRotation(),
            rotationArms.waitForMove(),
            // Telescopes pull up to just below high bar and rotation arms to behind the high bar
            telescopes.moveToPop(),
            telescopes.waitForRotationSafePosition(),
            rotationArms.moveToStow(),
            rotationArms.waitForMove(),
            telescopes.waitForMove(), 
            // Pull up to high bar
            telescopes.moveToPull(),
            telescopes.waitForMove(),
            // Rotation arms above high bar
            rotationArms.moveToClimbGrab(),
            rotationArms.waitForMove(),
            // Telescopes up a bit to clear them off the high bar
            telescopes.moveToPop(),
            telescopes.waitForMove(),


            // TO TRAVERSAL BAR
            // Telescopes extend while rotation arms move back to catch traversal bar
            rotationArms.moveToClimbSwing(),
            telescopes.moveToSwing(),
            rotationArms.waitForMove(),
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
            // Telescopes up to above traversal bar
            telescopes.moveToFull(),
            telescopes.waitForMove(),
            // Rotation arms to contact the traversal bar with telescopes
            rotationArms.latchRotation(),
            rotationArms.waitForMove(),
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
            // Telescopes pull up to just below traversal bar and rotation arms to behind the traversal bar
            telescopes.moveToPop(),
            telescopes.waitForRotationSafePosition(),
            rotationArms.moveToStow(),
            rotationArms.waitForMove(),
            telescopes.waitForMove(), 
            // Pull up to traversal bar
            telescopes.moveToPull(),
            telescopes.waitForMove()

        );
    }
}
