package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class VisionIOComp implements VisionIO {
    private double captureTimestamp = 0.0;
    private double[] cornerX = new double[] {};
    private double[] cornerY = new double[] {};
    private boolean simpleValid = false;
    private double simpleAngle = 0.0;

    private final NetworkTableEntry ledEntry = NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("ledMode");
    private final NetworkTableEntry validEntry =
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    private final NetworkTableEntry latencyEntry =
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
    private final NetworkTableEntry dataEntry = NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("tcornxy");
    private final NetworkTableEntry simpleAngleEntry =
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");

    public VisionIOComp() {
        latencyEntry.addListener(event -> {
            double timestamp = Logger.getInstance().getRealTimestamp()
                    - (latencyEntry.getDouble(0.0) / 1000.0);

            List<Double> cornerXList = new ArrayList<>();
            List<Double> cornerYList = new ArrayList<>();
            if (validEntry.getDouble(0.0) == 1.0) {
                boolean isX = true;
                for (double coordinate : dataEntry.getDoubleArray(new double[] {})) {
                    if (isX) {
                        cornerXList.add(coordinate);
                    } else {
                        cornerYList.add(coordinate);
                    }
                    isX = !isX;
                }
            }

            synchronized (VisionIOComp.this) {
                captureTimestamp = timestamp;
                cornerX =
                        cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
                cornerY =
                        cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
                simpleValid = validEntry.getDouble(0.0) == 1.0;
                simpleAngle = simpleAngleEntry.getDouble(0.0);
            }

        }, EntryListenerFlags.kUpdate);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        inputs.captureTimestamp = captureTimestamp;
        inputs.cornerX = cornerX;
        inputs.cornerY = cornerY;
        inputs.simpleValid = simpleValid;
        inputs.simpleAngle = simpleAngle;
    }

    @Override
    public void setLeds(boolean enabled) {
        ledEntry.forceSetDouble(enabled ? 3.0 : 1.0);
    }

    @Override
    public double getSimpleAngle() {
        return simpleAngle;
    }
}