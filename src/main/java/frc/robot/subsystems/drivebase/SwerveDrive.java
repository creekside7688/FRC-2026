package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveUtils;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivebase.module.ModuleIO;
import frc.robot.subsystems.drivebase.module.SparkOdometryThread;
import frc.robot.subsystems.drivebase.module.SwerveModule;
import frc.robot.subsystems.vision.Vision;

public class SwerveDrive extends SubsystemBase implements Vision.VisionConsumer {
    // Prevents writing to module IOInputs while reading data
    public static final Lock odometryLock = new ReentrantLock();
    // private final SwerveSetpointGenerator setpointGenerator;

    // Gyro
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Alert gyroDisconnectedAlert = new Alert("Gyro disconnected, using kinematics as fallback.",
            Alert.AlertType.kError);

    // Modules
    private final SwerveModule[] modules; // FL, FR, BL, BR
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    // Standard deviations for encoder, used for pose estimation - how trusted
    // encoder measurements are
    private static final Vector<N3> encoderStateDeviations = VecBuilder.fill(1.0, 1.0, 1.0);

    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private final PIDController rotationOverrideController = new PIDController(0.5, 0, 0);

    private LoggedNetworkBoolean rotationOverrideNT = new LoggedNetworkBoolean("Tuning/Drive/RotationOverride", false);
    private boolean rotationOverride = false;

    private LoggedNetworkNumber aimPoseX = new LoggedNetworkNumber("Tuning/Drive/AimX", Units.inchesToMeters(182.11));
    private LoggedNetworkNumber aimPoseY = new LoggedNetworkNumber("Tuning/Drive/AimY", Units.inchesToMeters(158.84));
    private LoggedNetworkNumber controllerP = new LoggedNetworkNumber("Tuning/Drive/ControllerP", 0);
    private LoggedNetworkNumber controllerI = new LoggedNetworkNumber("Tuning/Drive/ControllerI", 0);
    private LoggedNetworkNumber controllerD = new LoggedNetworkNumber("Tuning/Drive/ControllerD", 0);

    private Translation2d rotationOverridePoint;

    public SwerveDrive(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        modules = new SwerveModule[] {
                new SwerveModule(fl, "FL"),
                new SwerveModule(fr, "FR"),
                new SwerveModule(bl, "BL"),
                new SwerveModule(br, "BR")
        };

        this.gyroIO = gyro;
        this.zeroHeading();

        SparkOdometryThread.getInstance().start();

        this.poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.SWERVE_KINEMATICS,
                rawGyroRotation,
                lastModulePositions,
                Pose2d.kZero, // Cameras update pose on field, initialize to (0, 0)
                encoderStateDeviations,
                VecBuilder.fill(0.0, 0.0, 0.0) // Filler, not actually used
        );

        rotationOverrideController.enableContinuousInput(0, 2 * Math.PI);
        rotationOverrideController.setTolerance(0.3);

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::driveRelative,
                AutonomousConstants.pfc,
                AutonomousConstants.ROBOT_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

        // setpointGenerator = new
        // SwerveSetpointGenerator(AutonomousConstants.ROBOT_CONFIG,
        // AngularVelocity.ofBaseUnits(540, DegreesPerSecond));
    }

    @Override
    public void periodic() {
        rotationOverrideController.setP(controllerP.get());
        rotationOverrideController.setI(controllerI.get());
        rotationOverrideController.setD(controllerD.get());

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = DriveConstants.SWERVE_KINEMATICS.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
            rotationOverridePoint = new Translation2d(aimPoseX.get(), aimPoseY.get());
            rotationOverride = rotationOverrideNT.get();
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);

        Logger.recordOutput("SwerveStates/AutoLock", rotationOverride);
    }

    /**
     * Drives the robot using controller input.
     * 
     * @param xInput Controls forward/backward movement relative to alliance
     *               station. Positive -> forward movement
     * @param yInput Controls left/right movement relative to alliance station.
     *               Positive -> leftwards movement
     * @param rInput controls left/right rotation relative to alliance station.
     *               Positive -> CCW rotation
     */

    public void joystickDrive(double xInput, double yInput, double rInput) {
        Translation2d linearVelocity = SwerveUtils.GetLinearVelocityFromRawJoysticks(xInput, yInput);
        Logger.recordOutput("SwerveStates/Unoptimized/RawXLinearVelocity", linearVelocity.getX());
        Logger.recordOutput("SwerveStates/Unoptimized/RawYLinearVelocity", linearVelocity.getY());

        if (rotationOverride) {
        rInput = rotationOverrideController.calculate(SwerveUtils.wrapAngle(SwerveUtils.lookAtPoint(rotationOverridePoint, this.getPose().getTranslation()).getRadians()), SwerveUtils.wrapAngle(getRotation2d().getRadians())) / Math.PI;
        } else {
            rInput = MathUtil.applyDeadband(rInput, OperatorConstants.DEADBAND);
            rInput = Math.copySign(rInput * rInput, rInput);
        }

        Logger.recordOutput("SwerveStates/Unoptimized/RawRotationalVelocity", rInput);

        // Inputs are [-1, 1] - scale the percentages by max speed to get speeds
        ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                linearVelocity.getY() * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
                rInput * (!rotationOverride ? DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND : 1));

        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? getRotation2d().plus(Rotation2d.kPi) : getRotation2d());

        ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        Logger.recordOutput("SwerveStates/Unoptimized/RawDesiredModuleStates", setpointStates);
        Logger.recordOutput("SwerveStates/Unoptimized/RawDesiredChassisSpeeds", speeds);

        // Mutates state!
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/Optimized/DesiredModuleStates", setpointStates);
        Logger.recordOutput("SwerveStates/Optimized/DesiredChassisSpeeds",
                DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(setpointStates));
    }

    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate filters to control acceleration.
    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.ROTATION_SLEW_RATE);

    private double previousTime = WPIUtilJNI.now() * 1e-6;

    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean limitSpeed, boolean fieldRelative,
            boolean rateLimit) {

        // Cube the inputs for fine control at low speeds.

        if (xSpeed < 0)
            xSpeed = -Math.pow(Math.abs(xSpeed), 0.5);
        else
            xSpeed = Math.pow(Math.abs(xSpeed), 0.5);

        if (ySpeed < 0)
            ySpeed = -Math.pow(Math.abs(ySpeed), 0.5);
        else
            ySpeed = Math.pow(Math.abs(ySpeed), 0.5);

        /*
         * if (rSpeed < 0) rSpeed = - Math.pow(Math.abs(rSpeed), 0.5);
         * else rSpeed = Math.pow(Math.abs(rSpeed), 0.5);
         */

        /*
         * xSpeed = Math.pow(xSpeed, 1);
         * ySpeed = Math.pow(ySpeed, 1);
         */
        rSpeed = Math.pow(rSpeed, 1);

        SmartDashboard.putNumber("xTransformed", xSpeed);
        SmartDashboard.putNumber("yTransformed", ySpeed);
        SmartDashboard.putNumber("rTransformed", rSpeed);

        double xSpeedCommand;
        double ySpeedCommand;

        // If we want to ratelimit
        if (rateLimit) {
            // Get the elapsed time since the last period
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;

            // Convert the inputs to polar coordinates
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            double directionSlewRate;

            // If we are moving
            if (currentTranslationMagnitude != 0.0) {
                // Apply a slew rate.
                directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMagnitude);

                // If we are not moving
            } else {
                // Set an infinite slew rate
                directionSlewRate = 500.0;
            }

            // Find the minimum difference between the input and current direction
            double angleDif = SwerveUtils.angleDifference(inputTranslationDirection, currentTranslationDirection);

            // If the difference is less than 0.45 radians
            if (angleDif < (0.45 * Math.PI)) {
                // Step towards the input direction
                currentTranslationDirection = SwerveUtils
                        .stepTowardsCircular(currentTranslationDirection, inputTranslationDirection,
                                directionSlewRate * elapsedTime);

                // Limit the magnitude
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);

                // If the difference is greater than 0.85 radians
            } else if (angleDif > 0.85 * Math.PI) {
                // If the robot is moving
                if (currentTranslationMagnitude > 1e-4) {
                    // Remove the magnitude
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);

                    // Otherwise
                } else {
                    // Wrap the angle and calcualte
                    currentTranslationDirection = SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }

                // Otherwise
            } else {
                // Step towards the input direction, but remove the magnitude.
                currentTranslationDirection = SwerveUtils
                        .stepTowardsCircular(currentTranslationDirection, inputTranslationDirection,
                                directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
            }

            previousTime = currentTime;

            // Calculate the commanded speeds
            xSpeedCommand = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommand = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(rSpeed);

        } else {
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            currentRotation = rSpeed;
        }

        // Convert the speeds into percentages of the maximum speed.
        double xSpeedDelivered = xSpeedCommand
                * (limitSpeed ? DriveConstants.MAXIMUM_LIMITED_SPEED_METRES_PER_SECOND
                        : DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        double ySpeedDelivered = ySpeedCommand
                * (limitSpeed ? DriveConstants.MAXIMUM_LIMITED_SPEED_METRES_PER_SECOND
                        : DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        double rotDelivered = currentRotation
                * (limitSpeed ? DriveConstants.MAXIMUM_LIMITED_ANGULAR_SPEED_RADIANS_PER_SECOND
                        : DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND);

        // Calculate the desired module states based on if we are driving field relative
        // or not.
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                this.getRotation2d())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        // Desaturate the wheel speeds to prevent any speeds from exceeding the maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
        }

        Logger.recordOutput("SwerveStates/Optimized/DesiredModuleStates", swerveModuleStates);
        Logger.recordOutput("SwerveStates/Optimized/DesiredChassisSpeeds",
                DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(swerveModuleStates));
    }

    // public void enableRotationOverride(Translation2d point) {
    //     rotationOverride = true;
    //     this.rotationOverridePoint = point;
    // }

    // public void disableRotationOverride() {
    //     rotationOverride = false;
    // }

    /**
     * Drives the robot using field relative chassis speeds.
     */
    public void driveRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Returns the current state of the swerve drive in the form of a chassis speeds
     * object.
     */
    @AutoLogOutput(key = "SwerveStates/Measured/MeasuredChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "SwerveStates/Measured/MeasuredModuleStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    /**
     * Returns the current state of the swerve drive in the form of a swerve module
     * state array.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    /**
     * Returns the estimated position.
     */
    @AutoLogOutput(key = "Odometry/RobotPose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current rotation of the robot.
     */
    public Rotation2d getRotation2d() {
        return getPose().getRotation();
    }

    /**
     * Returns how fast the robot is turning in degrees per second.
     */
    public double getTurnRate() {
        return gyroInputs.yawVelocityDegreesPerSec;
    }

    /**
     * Resets odometry to a specified pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(this.getRotation2d(), this.getModulePositions(), pose);
    }

    /**
     * Sets the module states of the swerve drive.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    /**
     * Zeros the heading of the robot.
     */
    public void zeroHeading() {
        gyroIO.reset();
    }

    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}
