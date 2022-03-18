package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class ForceSetPosition extends CommandBase {

    private final Turret turret;
    private final Rotation2d desiredRotation;

    public ForceSetPosition(Turret turret, Rotation2d desiredRotation) {

        this.turret = turret;
        this.desiredRotation = desiredRotation;

        addRequirements(turret);

    }

    @Override
    public void initialize() {

        turret.setPositionGoal(desiredRotation);

    }

    @Override
    public boolean isFinished() {

        return turret.atGoal();

    }

    @Override
    public void end(boolean isInterrupted) {

        turret.setVoltage(0);

    }
    
}
