package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.util.CircleFitter;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private final VisionIO io;

    private double lastCaptureTimestamp = 0.0;
    private boolean ledsOn = true;

    private static final double vpw =
            2.0 * Math.tan(VisionConstants.fovHorizontal.getRadians() / 2.0);
    private static final double vph =
            2.0 * Math.tan(VisionConstants.fovVertical.getRadians() / 2.0);

    private static final int minTargetCount = 2; // Minimum number of tape targets to fit a circle
    private static final double circleFitPrecision = 0.01;

    public Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        // If there is no new frame, do nothing
        //if (inputs.captureTimestamp == lastCaptureTimestamp) return;
        lastCaptureTimestamp = inputs.captureTimestamp;

        // Grab target count from corners array.  If LEDs are off, force
        // no targets
        int targetCount = ledsOn ? inputs.cornerX.length / 4 : 0;

        // Stop if we don't have enough targets
        if (targetCount < minTargetCount) return;

        List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
        for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
            List<VisionPoint> corners = new ArrayList<>();
            double totalX = 0.0, totalY = 0.0;
            for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
                if (i < inputs.cornerX.length && i < inputs.cornerY.length) {
                    corners.add(new VisionPoint(inputs.cornerX[i], inputs.cornerY[i]));
                    totalX += inputs.cornerX[i];
                    totalY += inputs.cornerY[i];
                }
            }

            VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
            corners = sortCorners(corners, targetAvg);

            for (int i = 0; i < corners.size(); i++) {
                Translation2d translation = solveCameraToTargetTranslation(
                        corners.get(i), i < 2 ? VisionConstants.visionTargetHeightUpper
                                : VisionConstants.visionTargetHeightLower,
                        VisionConstants.cameraHeightM, Rotation2d.fromDegrees(VisionConstants.floorToCameraAngleDeg.get()));
                if (translation != null) {
                    cameraToTargetTranslations.add(translation);
                }
            }
        }

        // Stop if there aren't enough detected corners
        if (cameraToTargetTranslations.size() < minTargetCount * 4) return;

        Translation2d cameraToTargetTranslation = CircleFitter.fit(VisionConstants.visionTargetDiameter / 2.0,
                cameraToTargetTranslations, circleFitPrecision);

        Logger.getInstance().recordOutput("Vision/TargetDistanceIn", Units.metersToInches(cameraToTargetTranslation.getNorm()));
        Logger.getInstance().recordOutput("Vision/CameraToTargetIn", new double[] {Units.metersToInches(cameraToTargetTranslation.getX()), Units.metersToInches(cameraToTargetTranslation.getY()), 0});

        // Inform RobotState of our observations
        RobotState.getInstance().recordVisionObservations(lastCaptureTimestamp, cameraToTargetTranslation);
    }

    private List<VisionPoint> sortCorners(List<VisionPoint> corners,
                                          VisionPoint average) {

        // Find top corners
        Integer topLeftIndex = null;
        Integer topRightIndex = null;
        double minPosRads = Math.PI;
        double minNegRads = Math.PI;
        for (int i = 0; i < corners.size(); i++) {
            VisionPoint corner = corners.get(i);
            double angleRad =
                    new Rotation2d(corner.x - average.x, average.y - corner.y)
                            .minus(Rotation2d.fromDegrees(90)).getRadians();
            if (angleRad > 0) {
                if (angleRad < minPosRads) {
                    minPosRads = angleRad;
                    topLeftIndex = i;
                }
            } else {
                if (Math.abs(angleRad) < minNegRads) {
                    minNegRads = Math.abs(angleRad);
                    topRightIndex = i;
                }
            }
        }

        // Find lower corners
        Integer lowerIndex1 = null;
        Integer lowerIndex2 = null;
        for (int i = 0; i < corners.size(); i++) {
            boolean alreadySaved = false;
            if (topLeftIndex != null) {
                if (topLeftIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (topRightIndex != null) {
                if (topRightIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (!alreadySaved) {
                if (lowerIndex1 == null) {
                    lowerIndex1 = i;
                } else {
                    lowerIndex2 = i;
                }
            }
        }

        // Combine final list
        List<VisionPoint> newCorners = new ArrayList<>();
        if (topLeftIndex != null) {
            newCorners.add(corners.get(topLeftIndex));
        }
        if (topRightIndex != null) {
            newCorners.add(corners.get(topRightIndex));
        }
        if (lowerIndex1 != null) {
            newCorners.add(corners.get(lowerIndex1));
        }
        if (lowerIndex2 != null) {
            newCorners.add(corners.get(lowerIndex2));
        }
        return newCorners;
    }

    private Translation2d solveCameraToTargetTranslation(VisionPoint corner,
                                                         double goalHeight, double cameraHeight, Rotation2d floorToCamera) {

        double halfWidthPixels = VisionConstants.widthPixels / 2.0;
        double halfHeightPixels = VisionConstants.heightPixels / 2.0;
        double nY = -((corner.x - halfWidthPixels) / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels) / halfHeightPixels);

        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ)
                .rotateBy(floorToCamera);
        double x = xzPlaneTranslation.getX();
        double y = vpw / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = cameraHeight - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return new Translation2d(distance * angle.getCos(),
                    distance * angle.getSin());
        }
        return null;
    }

    public static class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class TimestampedTranslation2d {
        public final double timestamp;
        public final Translation2d translation;

        public TimestampedTranslation2d(double timestamp,
                                        Translation2d translation) {
            this.timestamp = timestamp;
            this.translation = translation;
        }
    }
}
