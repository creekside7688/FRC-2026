package frc.robot.subsystems.drivebase.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;

public class VisionIOSim extends VisionIOLimelight {
    private static VisionSystemSim visionSim;
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final Supplier<Pose2d> poseSupplier;

    private final PhotonCameraSim cameraSim;
    

    public VisionIOSim(Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
        super(robotToCamera);
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagFieldLayout);
        }

        this.poseSupplier = poseSupplier;

        SimCameraProperties cameraProperties = new SimCameraProperties()
            .setCalibration(640, 480, Rotation2d.fromDegrees(100))
            .setCalibError(0.25, 0.08)
            .setFPS(20)
            .setAvgLatencyMs(35)
            .setLatencyStdDevMs(5);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        Rotation3d rotation = new Rotation3d(0, 0, 0);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, robotToCamera);

    }
    
    @Override
    public void periodic() { 
        visionSim.update(poseSupplier.get());
        super.periodic();
    }
}
