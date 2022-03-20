package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class ForceSetPosition extends CommandBase {

    private final Turret turret;
    private final Vision vision;
    private final Rotation2d desiredRotation;

    public ForceSetPosition(Turret turret, Vision vision, Rotation2d desiredRotation) {

        this.turret = turret;
        this.vision = vision;
        this.desiredRotation = desiredRotation;

        addRequirements(turret, vision);

    }

    @Override
    public void initialize() {

        vision.turnOnLeds();
        turret.setPositionGoal(desiredRotation);

    }

    @Override
    public void end(boolean isInterrupted) {

        turret.setVoltage(0);
        vision.turnOffLeds();

    }
    
}
