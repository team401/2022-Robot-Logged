package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.rotationarms.RotationArms;
import frc.robot.subsystems.telescopes.TelescopesSubsystem;

public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(TelescopesSubsystem telescopes, RotationArms rotationArms, XboxController gamepad) {

        addRequirements(telescopes, rotationArms);

        addCommands( 
            // To Mid Bar
            telescopes.moveToPull().alongWith(rotationArms.moveToStow()) // Start moving hooks down and make sure rotation arms are out of the way
            .andThen(telescopes.waitForMove())
            .andThen(rotationArms.moveToClimbGrab()) // Move the rotation arms into the position above the rung
            .andThen(rotationArms.waitForMove())
            // To High Bar
            .andThen(telescopes.moveToPop()) // Move the telescopes up a bit to clear them off the rung
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
            .andThen(telescopes.moveToPop()) // Move the telescopes up a bit to clear them off the rung
            .andThen(telescopes.waitForMove())
            .andThen(new WaitCommand(0.5))
            .andThen(rotationArms.moveToClimbSwing()) // Swing rotation arms to behind the high bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToFull()) // Extend the telescopes to full
            .andThen(telescopes.waitForMove()) 
            .andThen(rotationArms.latchRotation()) // Move rotation arms to contact the high bar
            .andThen(rotationArms.waitForMove())
            .andThen(telescopes.moveToPop()) // Retract telescopes to just below high bar
            .andThen(telescopes.waitForRotationSafePosition() // Wait for when the rotation arms have cleared the high bar
                .andThen(rotationArms.moveToStow() // Move the rotation arms to the back position
                .andThen(rotationArms.waitForMove())))
            .andThen(telescopes.waitForMove()) 
            .andThen(telescopes.moveToPull()) // Fully retract telescopes to be on high bar
            .andThen(telescopes.waitForMove())
        );
    }
}
