package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.util.PicoColorSensor.RawColor;

public interface TowerIO {
    public static class TowerIOInputs implements LoggableInputs {
        public boolean topSensor;
        public boolean bottomSensor;

        public double conveyorCurrent;
        public double indexCurrent;

        public RawColor detectedColor;

        //How the logger gets data
        @Override
        public void toLog(LogTable table) {
            table.put("TopSensor", topSensor);
            table.put("BottomSensor", bottomSensor);
            table.put("ConveyorCurrent", conveyorCurrent);
            table.put("IndexCurrent", indexCurrent);
            table.put("DetectedColor", new int[] {detectedColor.red, detectedColor.green, detectedColor.blue, detectedColor.ir});

        }

        @Override
        public void fromLog(LogTable table) {
            topSensor = table.getBoolean("TopSensor", topSensor);
            bottomSensor = table.getBoolean("BottomSensor", bottomSensor);
            conveyorCurrent = table.getDouble("ConveyorCurrent", conveyorCurrent);
            indexCurrent = table.getDouble("IndexCurrent", indexCurrent);
            int[] arr = table.getIntegerArray("DetectedColor", new int[] {detectedColor.red, detectedColor.green, detectedColor.blue, detectedColor.ir});
            detectedColor = new RawColor(arr[0], arr[1], arr[2], arr[3]);

        }

    }

    void updateInputs(TowerIOInputs inputs);
    void setConveyorPercent(double percent);
    void setIndexWheelsPercent(double percent);
    
}
