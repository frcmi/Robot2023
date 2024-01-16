package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionEstimationSubsystem extends SubsystemBase {
    private PhotonPoseEstimator poseEstimator;
    private Optional<EstimatedRobotPose> lastPose;

    private final PhotonCamera camera = new PhotonCamera("USB_Camera");

    public PositionEstimationSubsystem() {
        // todo: mount & measure
        var robotToCamera = new Transform3d(new Translation3d(0.5, 0.5, 0.5), new Rotation3d(0, 0, 0));

        try {
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
                    robotToCamera);
        } catch (IOException exc) {
            System.out.println("Failed to load april tag field layout: " + exc.toString());
            poseEstimator = null;
        }

        poseEstimator.setReferencePose(new Pose2d(0, 0, new Rotation2d(0)));
        lastPose = poseEstimator.update();
    }

    @Override
    public void periodic() {
        if (poseEstimator != null) {
            if (lastPose.isPresent()) {
                poseEstimator.setReferencePose(lastPose.get().estimatedPose);
            }

            lastPose = poseEstimator.update();
        }

        SmartDashboard.putString("Pose", lastPose.toString());
    }

    public PhotonTrackedTarget getTargets() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }

        return null;
    }

    public Optional<Pose3d> getPose() {
        Optional<Pose3d> pose = Optional.empty();
        if (lastPose.isPresent()) {
            pose = Optional.of(lastPose.get().estimatedPose);
        }

        return pose;
    }
}
