package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class ManualControl extends CommandBase {

    private final Turret turret;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ManualControl(Turret turret, DoubleSupplier x, DoubleSupplier y) {

        this.turret = turret;
        this.xSupplier = x;
        this.ySupplier = y;

        addRequirements(turret);

    }

    @Override
    public void execute() {

        
        double filteredX = xFilter.calculate(xSupplier.getAsDouble());
        double filteredY = -yFilter.calculate(ySupplier.getAsDouble());
        filteredY = filteredY < 0 ? 0 : filteredY;
        double angle = filteredX != 0 ? Math.atan(filteredY / filteredX) : 0;
        angle += Math.PI / 2;

        if (angle > Math.PI/2)
            angle = -(Math.PI - angle);

        if (Math.sqrt(filteredX*filteredX + filteredY*filteredY) > 0.75 && filteredY != 0)
            turret.setPositionGoal(new Rotation2d(angle));
        
    }
    
}
