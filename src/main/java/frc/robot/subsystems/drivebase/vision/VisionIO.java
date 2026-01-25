package frc.robot.subsystems.drivebase.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
    PhotonPipelineResult getLatestResult();

    Optional<EstimatedRobotPose> getEstimatedPosition();

    record IOData() {};
}
