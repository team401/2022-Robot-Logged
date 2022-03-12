package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;

import frc.robot.subsystems.vision.Vision;
import frc.robot.RobotState;
import frc.robot.RobotState.AimingParameters;
import frc.robot.subsystems.turret.Turret;

public class Tracking extends CommandBase {
    
    private final Vision vision;
    private final Turret turret;

    private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public Tracking(Vision vision, Turret turret) {
        this.vision = vision;
        this.turret = turret;

        addRequirements(vision, turret);
    }

    @Override
    public void initialize() {
        filter.reset();
    }

    @Override
    public void execute() {

        AimingParameters params = RobotState.getInstance().getAimingParameters();
        double filteredAngle = filter.calculate(params.getTurretAngle().getRadians());
        turret.setPositionGoal(new Rotation2d(filteredAngle), params.getVelocityRadPerSec());
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        turret.setPositionGoal(new Rotation2d());
    }
}
