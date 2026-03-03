package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AutonomousConstants;
import frc.robot.subsystems.drivebase.SwerveDrive;

public class PositionPIDCommand extends Command {

    public SwerveDrive drive;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = AutonomousConstants.PATHFINDING_CONTROLLER;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private PositionPIDCommand(SwerveDrive drive, Pose2d goalPose) {
        this.drive = drive;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = drive.getPose().relativeTo(goalPose);

            boolean rotation =
                    MathUtil.isNear(0.0, diff.getRotation().getRotations(), Units.degreesToRotations(1), 0.0, 1.0);

            boolean position = diff.getTranslation().getNorm() < 0.01;

            boolean speed =
                    Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond)
                            < 0.1;

            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(0.1);
    }

    public static Command generateCommand(SwerveDrive swerve, Pose2d goalPose, double timeoutSeconds) {
        return new PositionPIDCommand(swerve, goalPose)
                .withTimeout(timeoutSeconds)
                .finallyDo(() -> {
                    swerve.runVelocity(new ChassisSpeeds(0, 0, 0));
                });
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        drive.runVelocity(mDriveController.calculateRobotRelativeSpeeds(drive.getPose(), goalState));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
