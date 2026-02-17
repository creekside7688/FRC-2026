package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.SwerveUtils;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.drivebase.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopDrive {

    private TeleopDrive() {}

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            SwerveDrive swerveDrive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = SwerveUtils.GetLinearVelocityFromRawJoysticks(
                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * DrivebaseConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                            linearVelocity.getY() * DrivebaseConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                            omega * DrivebaseConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND);
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    swerveDrive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                    ? swerveDrive.getRotation2d().plus(new Rotation2d(Math.PI))
                                    : swerveDrive.getRotation2d()));
                },
                swerveDrive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveWithRotationalOverride(
            SwerveDrive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        @SuppressWarnings("resource") // :sob:
        PIDController angleController =
                new PIDController(DrivebaseConstants.OVERRIDE_ANGLE_KP, 0.0, DrivebaseConstants.OVERRIDE_ANGLE_KD);
        angleController.enableContinuousInput(-Math.PI, Math.PI); // I love pid controllers

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity = SwerveUtils.GetLinearVelocityFromRawJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation2d().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * DrivebaseConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                                    linearVelocity.getY() * DrivebaseConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation2d().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation2d()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(
                        () -> angleController.setSetpoint(drive.getRotation2d().getRadians()));
    }
}
