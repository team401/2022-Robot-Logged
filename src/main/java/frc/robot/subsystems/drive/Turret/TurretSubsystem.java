package frc.robot.subsystems.drive.Turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.drive.Turret.TurretIO.TurretIOInputs;

public class TurretSubsystem extends SubsystemBase {

    private TurretIOInputs inputs; 
    private PIDController turretPID;

    private TurretIO turretIO;

    public TurretSubsystem(TurretIO turretIO) {

        this.turretIO = turretIO;
        inputs = new TurretIOInputs();

        turretIO.resetEncoderAbsolute();

        turretPID = new PIDController(
            SuperStructureConstants.turretkp.get(), 0, SuperStructureConstants.turretkd.get());

    }
    
    @Override
    public void periodic() {

        turretIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Turret", inputs);

        if (SuperStructureConstants.turretkp.hasChanged() || SuperStructureConstants.turretkd.hasChanged()) {

            turretPID.setP(SuperStructureConstants.turretkp.get());
            turretPID.setD(SuperStructureConstants.turretkd.get());

        }

    }

    
}
