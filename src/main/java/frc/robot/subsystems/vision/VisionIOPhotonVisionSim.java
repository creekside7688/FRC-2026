package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
      
    }

    var cameraProperties = SimCameraProperties.LL2_1280_720().setCalibration(1280, 720, Rotation2d.fromDegrees(79.1));
    cameraSim = new PhotonCameraSim(camera, cameraProperties, APRIL_TAG_FIELD_LAYOUT);
    cameraSim.enableDrawWireframe(false); // Resource intensive - disable if not needed
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}