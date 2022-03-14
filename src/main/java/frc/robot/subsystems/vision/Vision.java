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

/**
 * Subsystem implementing vision.  This subsystem takes in data sent over NetworkTables by the limelight pointed at the
 * target, sorts the detected corners of each piece of vision tape, finds the vector pointing from the camera to each
 * corner, and fits a circle with the known radius of the goal circle to the detected corners, ultimately calculating
 * the vector from the camera lens to the center of the goal.  This vector is then reported to RobotState using the
 * "recordVisionObservations" method.
 *
 * This subsystem also provides methods for turning the LEDs on and off.
 *
 * REQUIRE THIS SUBSYSTEM IN COMMANDS IF: you need to control the limelight LEDs.  Vision data is always recorded to
 *                                        RobotState automatically, so commands don't need to worry about that.
 */
public class Vision extends SubsystemBase {
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private final VisionIO io;

    // Timestamp in seconds that the last image was captured at
    private double lastCaptureTimestamp = 0.0;

    // Whether or not the LEDs should be on.
    private boolean ledsOn = false;

    // Field of view constants for the limelight at 1x zoom
    private static final double vpw =
            2.0 * Math.tan(VisionConstants.fovHorizontal.getRadians() / 2.0);
    private static final double vph =
            2.0 * Math.tan(VisionConstants.fovVertical.getRadians() / 2.0);

    // Minimum number of pieces of reflective tape that must be seen to constitute a target detection.
    private static final int minTargetCount = 2;

    // Precision that must be achieved by the circle fitter, in meters.
    private static final double circleFitPrecision = 0.01;

    // Constant latency applied to the timestamps we get from NetworkTables, accounting for latency in image capture
    // and transport over the network that cannot be measured in software.
    private static final double constantLatency = 0.06;

    public Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        io.setLeds(ledsOn);

        // If there is no new frame, do nothing
        if (inputs.captureTimestamp == lastCaptureTimestamp) return;
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
        RobotState.getInstance().recordVisionObservations(lastCaptureTimestamp - constantLatency, cameraToTargetTranslation);
    }

    /**
     * Sorts a list of corners found in the image to the expected order for vision processing.
     * @param corners The list of all corners
     * @param average The point that is the average of all corners (geometric center)
     * @return Sorted list of corners
     */
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

    /**
     * Given a list of corners, solves for the translation "camera to target", meaning the vector that comes out of
     * the camera's lens and points into the center of the target.
     *
     * This is done by using the known height of the camera and point of interest, and using the difference of them
     * combined with the camera's angle, field of view, and image resolution to correct for the perspective of the camera.
     *
     * @param corner The point to process
     * @param goalHeight The known height of the point in world space
     * @param cameraHeight The known height of the camera in world space
     * @param floorToCamera The rotation between the floor and the lens, in the floor plane frame of reference.
     * @return The vector from the camera to the target.
     */
    private Translation2d solveCameraToTargetTranslation(VisionPoint corner,
                                                         double goalHeight, double cameraHeight, Rotation2d floorToCamera) {

        // Compute constants for going from pixel space to camera space
        double halfWidthPixels = VisionConstants.widthPixels / 2.0;
        double halfHeightPixels = VisionConstants.heightPixels / 2.0;
        double nY = -((corner.x - halfWidthPixels) / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels) / halfHeightPixels);

        // Rotate by camera angle
        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ)
                .rotateBy(floorToCamera);
        double x = xzPlaneTranslation.getX();
        double y = vpw / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        // Perspective correction
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

    /**
     * Class representing pixel coordinates for a point of interest in the camera's image.
     */
    public static class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Turns on the limelight LEDs
     */
    public void turnOnLeds() {
        ledsOn = true;
    }

    /**
     * Turns off the limelight LEDs
     */
    public void turnOffLeds() {
        ledsOn = false;
    }
}
