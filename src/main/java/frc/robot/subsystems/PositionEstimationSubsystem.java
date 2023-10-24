package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionEstimationSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("photonvision");

    public PositionEstimationSubsystem() {
        // todo: configure...
    }
}
