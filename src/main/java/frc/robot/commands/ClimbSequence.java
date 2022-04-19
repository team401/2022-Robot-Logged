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
            // Telescopes up to above traversal bar
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
            telescopes.moveToFull(),
            telescopes.waitForMove(),
            // Rotation arms to contact the traversal bar with telescopes
            rotationArms.latchRotation(),
            rotationArms.waitForMove(),
            // Telescopes pull up to just below traversal bar and rotation arms to behind the traversal bar
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
            telescopes.moveToPop(),
            telescopes.waitForRotationSafePosition(),
            rotationArms.moveToStow(),
            rotationArms.waitForMove(),
            telescopes.waitForMove(), 
            // Pull up to traversal bar
            telescopes.moveToPull(),
            telescopes.waitForMove()

    
            /*// To Mid Bar
            telescopes.moveToPull().alongWith(rotationArms.moveToStow()) // Start moving hooks down and make sure rotation arms are out of the way
            .andThen(telescopes.waitForMove())
            .andThen(rotationArms.moveToClimbGrab()) // Move the rotation arms into the position above the bar
            .andThen(rotationArms.waitForMove())
            // To High Bar
            .andThen(telescopes.moveToPop()) // Move the telescopes up a bit to clear them off the bar
            .andThen(telescopes.waitForMove())
            .andThen(new WaitCommand(0.5))
            .andThen(rotationArms.moveToClimbSwing()) // Swing rotation arms to behind the high bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToFull()) // Extend the telescopes to full
            .andThen(telescopes.waitForMove())
            .andThen(rotationArms.latchRotation()) // Move rotation arms to contact the high bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToPop()) // Retract telescopes to just below high bar
            .andThen(telescopes.waitForMove()) 
            .andThen(rotationArms.moveToStow()) // Move rotation arms to behind the high bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToPull()) // Fully retract telescopes to be on high bar
            .andThen(telescopes.waitForMove()) //
            .andThen(rotationArms.moveToClimbGrab()) // Move the rotation arms to above the high bar
            .andThen(rotationArms.waitForMove())
            // To Traverse Bar
            .andThen(new WaitUntilCommand(() -> gamepad.getLeftBumperPressed())) // Wait for left bumper press to continue
            .andThen(telescopes.moveToPop()) // Move the telescopes up a bit to clear them off the bar
            .andThen(telescopes.waitForMove())
            .andThen(new WaitCommand(0.5))
            .andThen(rotationArms.moveToClimbSwing()) // Swing rotation arms to behind the traverse bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToFull()) // Extend the telescopes to full
            .andThen(telescopes.waitForMove()) 
            .andThen(rotationArms.latchRotation()) // Move rotation arms to contact the traverse bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToPop()) // Retract telescopes to just below traverse bar
            .andThen(telescopes.waitForRotationSafePosition() // Wait for when the rotation arms have cleared the traverse bar
                .andThen(rotationArms.moveToStow() // Move the rotation arms to the back position
                .andThen(rotationArms.waitForMove())))
            .andThen(telescopes.waitForMove()) 
            .andThen(telescopes.moveToPull()) // Fully retract telescopes to be on traverse bar
            .andThen(telescopes.waitForMove())*/
        );
    }
}
