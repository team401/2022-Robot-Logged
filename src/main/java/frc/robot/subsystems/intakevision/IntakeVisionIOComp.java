package frc.robot.subsystems.intakevision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeVisionIOComp implements IntakeVisionIO {

    private final NetworkTable table;
    private final NetworkTableEntry ledEntry;

    public IntakeVisionIOComp() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        ledEntry = table.getEntry("ledMode");
    }

	@Override
	public void updateInputs(IntakeVisionIOInputs inputs) {
		
        inputs.tx = table.getEntry("tx").getDouble(0.0);
        inputs.tv = table.getEntry("tv").getDouble(0.0);
        inputs.ta = table.getEntry("ta").getDouble(0.0);
		
	}

	@Override
	public void setLeds(boolean enabled) {
	    ledEntry.forceSetDouble(enabled ? 3.0 : 1.0);	
	}
    
}