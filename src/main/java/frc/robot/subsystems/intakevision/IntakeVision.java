package frc.robot.subsystems.intakevision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeVision extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera("photonvision"); // UPDATE to name of camera

    public double getTX() {
        return camera.getLatestResult().getBestTarget().getYaw();
    }

    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }
    
}
