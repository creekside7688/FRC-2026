package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.swerve.SwerveDrive;


import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.targeting.PhotonTrackedTarget;


public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera = new PhotonCamera( "7688Camera") ;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    private final Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);




    public Limelight() {
        SmartDashboard.putNumber("Target Yaw", 0);
        SmartDashboard.putNumber("Target Pitch", 0);
        SmartDashboard.putNumber("Target FID", 0);
        SmartDashboard.putBoolean("Robot Has targets?", false);
        //PortForwarder.add(5800, "photonvision.local", 5800);
    }
    
    /*public void getrobotpos() {
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            
        }
        
    
    
    }*/


    public void updatesd() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (target!=null) {
            SmartDashboard.putNumber("Target FID", target.getFiducialId());
            
        }
    }


    public Pose3d getEstimateAprilTag() {
        
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            
        } else {
            return null;
        }
    }
    

    /*public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }*/

    public double TargetYaw() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (target!=null) {
            SmartDashboard.putNumber("Target Yaw", target.getYaw());
            return target.getYaw();

        } else {
            return 0;
        }

    }

    public double TargetPitch() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (target!=null) {
            SmartDashboard.putNumber("Target Pitch", target.getPitch());
            return target.getPitch();
        } else {
            return 0;
        }
    }

    public int getAprilTag() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (target!=null) {
            SmartDashboard.putNumber("Target FID", target.getFiducialId());
            return target.getFiducialId();
        } else {

            return 0;
        }
    }

    public boolean CameraHasTargets() {
        var results = robotcamera.getLatestResult();
        SmartDashboard.putBoolean("Robot Has targets?", results.hasTargets());
        return results.hasTargets();
    }

    @Override
    public void periodic() {
    }

}