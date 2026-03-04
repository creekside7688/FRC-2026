package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SwerveUtils;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.drivebase.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

    private DriveCommands() {}

    private static final PIDController angleController =
            new PIDController(DrivebaseConstants.OVERRIDE_ANGLE_KP, 0.0, DrivebaseConstants.OVERRIDE_ANGLE_KD);

    private static final PIDController yController = new PIDController(4, 0, 0);

    private static final Trigger autoAlignAtSetpoint = new Trigger(() -> angleController.atSetpoint());
    private static final Trigger autoAlignDebounced = autoAlignAtSetpoint.debounce(0.2);

    static {
        angleController.enableContinuousInput(-Math.PI, Math.PI); // I love pid controllers
        angleController.setTolerance(0.1); // Radians
        yController.setTolerance(0.1); // Meters
    }

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

        // Construct command
        return Commands.startRun(
                () -> angleController.reset(),
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
                            isFlipped ? drive.getRotation2d().plus(new Rotation2d(Math.PI)) : drive.getRotation2d()));
                },
                drive);
    }

    public static Command joystickDriveWithTrenchAlign(SwerveDrive drive, DoubleSupplier xSupplier) {

        // Construct command
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            SwerveUtils.GetLinearVelocityFromRawJoysticks(xSupplier.getAsDouble(), 0);

                    // Calculate angular speed
                    double omega = angleController.calculate(
                            drive.getRotation2d().getRadians(),
                            getTrenchLockAngle(drive.getRotation2d().getDegrees())
                                    .getRadians());

                    // Convert to field relative speeds & send command
                    int flipFactor = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red
                            ? -1
                            : 1;
                    double robotPoseY = drive.getPose().getY();
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * DrivebaseConstants.MAXIMUM_SPEED_METRES_PER_SECOND * flipFactor,
                            yController.calculate(robotPoseY, getTrenchY(robotPoseY))
                                    * DrivebaseConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                            omega);
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation2d()));
                },
                drive);
    }

    private static Rotation2d getTrenchLockAngle(double rotationDegrees) {
        if (Math.abs(MathUtil.inputModulus(rotationDegrees - 90, -180, 180)) < 90) {
            return Rotation2d.kCCW_90deg;
        } else {
            return Rotation2d.kCW_90deg;
        }
    }

    private static double getTrenchY(double robotPose) {
        if (robotPose >= GameConstants.FIELD_WIDTH.div(2).in(Meters)) {
            return GameConstants.FIELD_WIDTH.minus(GameConstants.TRENCH_CENTER).in(Meters);
        }
        return GameConstants.TRENCH_CENTER.in(Meters);
    }

    public static class AutonomousHubAlign extends Command {
        private final SwerveDrive drive;
        private final Supplier<Rotation2d> sup;

        public AutonomousHubAlign(SwerveDrive drive, Supplier<Rotation2d> sup) {
            this.drive = drive;
            this.sup = sup;
            this.addRequirements(drive);
        }

        @Override
        public void initialize() {
            angleController.reset();
        }

        @Override
        public void execute() {
            double omega = angleController.calculate(
                    drive.getRotation2d().getRadians(), sup.get().getRadians());
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);

            boolean isFlipped = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds, isFlipped ? drive.getRotation2d().plus(new Rotation2d(Math.PI)) : drive.getRotation2d()));
        }

        @Override
        public void end(boolean interrupted) {
            drive.runVelocity(new ChassisSpeeds());
        }

        public boolean isFinished() {
            return autoAlignDebounced.getAsBoolean();
        }
    }
}
