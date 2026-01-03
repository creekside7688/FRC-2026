<<<<<<< HEAD
package frc.robot.commands;

import java.util.Optional;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.Timer;

/**
 * AutoAlign
 */

public class AutoAlign extends Command {
    private final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.TRANSLATE_CONTROLLER.kV_MAX, VisionConstants.TRANSLATE_CONTROLLER.kA_MAX);

    private final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.TRANSLATE_CONTROLLER.kV_MAX, VisionConstants.TRANSLATE_CONTROLLER.kA_MAX);

    private final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.THETA_CONTROLLER.kV_MAX, VisionConstants.THETA_CONTROLLER.kA_MAX);

    private final Limelight limelight;

    private final SwerveDrive sd;

    private final PIDController xController = new PIDController(
            VisionConstants.TRANSLATE_CONTROLLER.kP,
            VisionConstants.TRANSLATE_CONTROLLER.kI,
            VisionConstants.TRANSLATE_CONTROLLER.kD);

    private final PIDController yController = new PIDController(
            VisionConstants.TRANSLATE_CONTROLLER.kP,
            VisionConstants.TRANSLATE_CONTROLLER.kI,
            VisionConstants.TRANSLATE_CONTROLLER.kD);

    private final PIDController thetaController = new PIDController(
            VisionConstants.THETA_CONTROLLER.kP,
            VisionConstants.THETA_CONTROLLER.kI,
            VisionConstants.THETA_CONTROLLER.kD);

    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(VisionConstants.GOAL_X, VisionConstants.GOAL_Y, VisionConstants.GOAL_Z),
            new Rotation3d(0.0, 0.0, Math.PI));

    private PhotonTrackedTarget lastTarget;

    public AutoAlign(SwerveDrive sd, Limelight limelight) {
        xController.setTolerance(VisionConstants.TRANSLATE_CONTROLLER.TOLERANCE);
        yController.setTolerance(VisionConstants.TRANSLATE_CONTROLLER.TOLERANCE);
        thetaController.setTolerance(VisionConstants.THETA_CONTROLLER.TOLERANCE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.limelight = limelight;
        this.sd = sd;
        addRequirements(limelight, sd);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = sd.getPose();

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    public void updateSD() {
        SmartDashboard.putData("PIDx", xController);
        SmartDashboard.putNumber("PIDx Set", xController.getSetpoint());
        SmartDashboard.putData("PIDy", yController);
        SmartDashboard.putData("PIDt", thetaController);

        SmartDashboard.putNumber("PoseX", sd.getPose().getX());
        SmartDashboard.putNumber("PoseY", sd.getPose().getY());
        SmartDashboard.putNumber("PoseT", sd.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = sd.getPose();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonPipelineResult result = limelight.getLatestResult();
        if (result.hasTargets()) {
            Optional<PhotonTrackedTarget> potentialTarget = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == 3 || t.getFiducialId() == 5)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= VisionConstants.HIGHEST_AMBIGUITY
                            && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (potentialTarget.isPresent()) {
                var target = potentialTarget.get();
                lastTarget = target;

                Pose3d cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_CAM);

                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d targetPose = cameraPose.transformBy(camToTarget);

                Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xController.setSetpoint(goalPose.getX());
                yController.setSetpoint(goalPose.getY());
                thetaController.setSetpoint(goalPose.getRotation().getRadians());
            }
        }

        if (lastTarget == null) {
            sd.driveRelative(
                    new ChassisSpeeds(
                            0,
                            0,
                            0));
        } else {
            double xSpeed = xController.calculate(robotPose.getX());
            if (xController.atSetpoint())
                xSpeed = 0;

            double ySpeed = yController.calculate(robotPose.getY());
            if (yController.atSetpoint())
                ySpeed = 0;

            double thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
            if (thetaController.atSetpoint())
                thetaSpeed = 0;

            sd.driveRelative(
                    new ChassisSpeeds(
                            xSpeed,
                            ySpeed,
                            thetaSpeed));
        }

        SmartDashboard.putBoolean("at setpoint", xController.atSetpoint() && yController.atSetpoint() &&
                thetaController.atSetpoint());
        updateSD();
    }

    @Override
    public void end(boolean interrupted) {
        sd.drive(0, 0, 0, false, false, false);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() &&
                thetaController.atSetpoint();
    }
}
=======
package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * AutoAlign
 */

public class AutoAlign extends Command {
//    private final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
//            VisionConstants.TRANSLATE_CONTROLLER.kV_MAX, VisionConstants.TRANSLATE_CONTROLLER.kA_MAX);
//
//    private final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
//            VisionConstants.TRANSLATE_CONTROLLER.kV_MAX, VisionConstants.TRANSLATE_CONTROLLER.kA_MAX);
//
//    private final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
//            VisionConstants.THETA_CONTROLLER.kV_MAX, VisionConstants.THETA_CONTROLLER.kA_MAX);

    private final Limelight limelight;

    private final SwerveDrive sd;

    private final PIDController xController = new PIDController(
            VisionConstants.TRANSLATE_CONTROLLER.kP,
            VisionConstants.TRANSLATE_CONTROLLER.kI,
            VisionConstants.TRANSLATE_CONTROLLER.kD);

    private final PIDController yController = new PIDController(
            VisionConstants.TRANSLATE_CONTROLLER.kP,
            VisionConstants.TRANSLATE_CONTROLLER.kI,
            VisionConstants.TRANSLATE_CONTROLLER.kD);

    private final PIDController thetaController = new PIDController(
            VisionConstants.THETA_CONTROLLER.kP,
            VisionConstants.THETA_CONTROLLER.kI,
            VisionConstants.THETA_CONTROLLER.kD);

    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(VisionConstants.GOAL_X, VisionConstants.GOAL_Y, VisionConstants.GOAL_Z),
            new Rotation3d(0.0, 0.0, Math.PI));

    private PhotonTrackedTarget lastTarget;

    public AutoAlign(SwerveDrive sd, Limelight limelight) {
        xController.setTolerance(VisionConstants.TRANSLATE_CONTROLLER.TOLERANCE);
        yController.setTolerance(VisionConstants.TRANSLATE_CONTROLLER.TOLERANCE);
        thetaController.setTolerance(VisionConstants.THETA_CONTROLLER.TOLERANCE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.limelight = limelight;
        this.sd = sd;
        addRequirements(limelight, sd);
    }

    @Override
    public void initialize() {
        lastTarget = null;

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    public void updateSD() {
        SmartDashboard.putData("PIDx", xController);
        SmartDashboard.putData("PIDy", yController);
        SmartDashboard.putData("PIDt", thetaController);

        SmartDashboard.putNumber("PoseX", sd.getPose().getX());
        SmartDashboard.putNumber("PoseY", sd.getPose().getY());
        SmartDashboard.putNumber("PoseT", sd.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = sd.getPose();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonPipelineResult result = limelight.getLatestResult();
        if (result.hasTargets()) {
            Optional<PhotonTrackedTarget> potentialTarget = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == 3 || t.getFiducialId() == 5)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= VisionConstants.HIGHEST_AMBIGUITY
                            && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (potentialTarget.isPresent()) {
                var target = potentialTarget.get();
                lastTarget = target;

                Pose3d cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_CAM);

                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d targetPose = cameraPose.transformBy(camToTarget);

                Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xController.setSetpoint(goalPose.getX());
                yController.setSetpoint(goalPose.getY());
                thetaController.setSetpoint(goalPose.getRotation().getRadians());
            }
        }

        if (lastTarget == null) {
            sd.driveRelative(
                    new ChassisSpeeds(
                            0,
                            0,
                            0));
        } else {
            double xSpeed = xController.calculate(robotPose.getX());
            if (xController.atSetpoint())
                xSpeed = 0;

            double ySpeed = yController.calculate(robotPose.getY());
            if (yController.atSetpoint())
                ySpeed = 0;

            double thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
            if (thetaController.atSetpoint())
                thetaSpeed = 0;

            sd.driveRelative(
                    new ChassisSpeeds(
                            xSpeed > VisionConstants.TRANSLATE_CONTROLLER.kV_MAX  ? VisionConstants.TRANSLATE_CONTROLLER.kV_MAX : xSpeed,
                            ySpeed > VisionConstants.TRANSLATE_CONTROLLER.kV_MAX  ? VisionConstants.TRANSLATE_CONTROLLER.kV_MAX : ySpeed,
                            thetaSpeed > VisionConstants.THETA_CONTROLLER.kV_MAX  ? VisionConstants.THETA_CONTROLLER.kV_MAX : thetaSpeed
                            ));
        }

        SmartDashboard.putBoolean("at setpoint", xController.atSetpoint() && yController.atSetpoint() &&
                thetaController.atSetpoint());
        updateSD();
    }

    @Override
    public void end(boolean interrupted) {
        sd.drive(0, 0, 0, false, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
>>>>>>> b21618d (working driveToPose + global robot positioning (marcus' basement edition))
