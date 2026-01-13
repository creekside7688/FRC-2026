package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Limelight extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult latestPipeline;

     private final PhotonPoseEstimator photonPoseEstimator;

    public Limelight() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResult() {
        return latestPipeline;
    }

    public Optional<EstimatedRobotPose> getEstimatedPosition() {
        return photonPoseEstimator.update(latestPipeline);
        
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();

        if (results.size() > 0) {
            latestPipeline = results.get(results.size() - 1);
        }
    }
}
