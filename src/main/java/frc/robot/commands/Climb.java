package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.telescopes.TelescopesSubsystem;

public class Climb extends CommandBase {

    private final TelescopesSubsystem telescopes;

    private final Timer leftHomeTimer = new Timer();
    private final Timer rightHomeTimer = new Timer();

    public Climb(TelescopesSubsystem tele) {
        telescopes = tele;
        addRequirements(telescopes);
    }

    @Override
    public void initialize() {
        leftHomeTimer.reset();
        leftHomeTimer.start();

        rightHomeTimer.reset();
        rightHomeTimer.start();

        telescopes.setOverride(true);
    }

    @Override
    public void execute() {
        telescopes.setLeftPercent(-0.7);
        telescopes.setRightPercent(-0.7);

        if (Math.abs(telescopes.getLeftVelocityRadPerS()) > ClimberConstants.telescopeHomingThresholdRadPerS) {
            leftHomeTimer.reset();  
        }
        if (Math.abs(telescopes.getRightVelocityRadPerS()) > ClimberConstants.telescopeHomingThresholdRadPerS) {
            rightHomeTimer.reset();
        }
        if (leftHomeTimer.hasElapsed(ClimberConstants.homingTimeS)) {
            telescopes.setLeftVolts(-10);
        }
        if (rightHomeTimer.hasElapsed(ClimberConstants.homingTimeS)) {
            telescopes.setRightPercent(-10);
        }

    }

    @Override
    public boolean isFinished() {
        return leftHomeTimer.hasElapsed(ClimberConstants.homingTimeS) && rightHomeTimer.hasElapsed(ClimberConstants.homingTimeS);
    }

    @Override
    public void end(boolean isInterrupted) {
        telescopes.setDesiredPosition(0);
    }
}
