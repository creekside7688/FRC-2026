package frc.robot.subsystems.drivebase;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LocalADStarAK;
import frc.lib.SwerveUtils;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivebase.module.ModuleIO;
import frc.robot.subsystems.drivebase.module.SparkOdometryThread;
import frc.robot.subsystems.drivebase.module.SwerveModule;
import frc.robot.subsystems.vision.Vision;
public class SwerveDrive extends SubsystemBase implements Vision.VisionConsumer {
    public static final Lock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Alert gyroDisconnectedAlert = new Alert("Gyro disconnected, using kinematics as fallback.", Alert.AlertType.kError);

    private final SwerveModule[] modules;

    private SwerveModulePosition[] lastModulePositions =
    new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private static final Vector<N3> stateDeviations = VecBuilder.fill(1.0, 1.0, 1.0);

    private static final Vector<N3> visionMeasurementDeviations = VecBuilder.fill(1.0, 1.0, 1.0);

    private final SwerveDrivePoseEstimator poseEstimator;

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;

    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate filters to control acceleration.
    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.ROTATION_SLEW_RATE);

    private double previousTime = WPIUtilJNI.now() * 1e-6;
    
        private Rotation2d rawGyroRotation = Rotation2d.kZero;
    
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
                    Pose2d.kZero,
                    stateDeviations,
                    visionMeasurementDeviations);
    
            RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                config = null;
                e.printStackTrace();
            }
    
            AutoBuilder.configure(
                    this::getPose,
                    this::setPose,
                    this::getChassisSpeeds,
                    this::driveRelative,
                    AutonomousConstants.pfc,
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
    
            Pathfinding.setPathfinder(new LocalADStarAK());
            PathPlannerLogging.setLogActivePathCallback((activePath) -> {
                Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
            PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
            PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
                Logger.recordOutput("Odometry/TrajectoryCurrentPose", currentPose);
            });
        }
    
        @Override
        public void periodic() {
            odometryLock.lock();
    
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Drive/Gyro", gyroInputs);
    
            for(SwerveModule module : modules) {
                module.updateInputs();
            }
    
            odometryLock.unlock();
    
            double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
            int sampleCount = sampleTimestamps.length;
            for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
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
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);
    }

    /**
     * Drives the robot using controller input.
     */
    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean limitSpeed, boolean fieldRelative,
            boolean rateLimit) {
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
        // rSpeed = Math.pow(rSpeed, 1);

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
    }

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
     * Sets the wheels into an X formation to prevent movement. Use for defense or
     * when the robot needs to be stationary.
     */
    public void lockPosition() {
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void driveBackPose() {
        for (SwerveModule module : modules) {
            module.setDesiredState(new SwerveModuleState(0.5, Rotation2d.kZero));
        }
    }

    /**
     * Returns the current state of the swerve drive in the form of a chassis speeds
     * object.
     */
    @AutoLogOutput(key = "SwerveStates/MeasuredChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "SwerveStates/MeasuredModuleStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    @AutoLogOutput(key = "SwerveStates/DesiredModuleStates")
    public SwerveModuleState[] getDesiredStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getDesiredState();
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
    @AutoLogOutput(key = "Odometry/Robot")
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
     * Resets the position on the field to 0, 0, 0-degrees, with forward being
     * downfield. This resets what "forward" is for field oriented driving.
     */
    public void resetPose() {
        this.setPose(new Pose2d());
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

    public Command followPath(Pose2d endPose) {
        PathConstraints constraints = new PathConstraints(1.93, 9.5,
                2 * Math.PI, RadiansPerSecond.convertFrom(2152, DegreesPerSecond)); // The constraints for

        return AutoBuilder.pathfindToPose(endPose, constraints, 1.50);
    }

    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}

// swerveDrive
