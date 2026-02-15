package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;

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
    // private final SwerveSetpointGenerator setpointGenerator;

    // Prevents writing to module IOInputs while reading data
    public static final Lock odometryLock = new ReentrantLock();

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

    // Standard deviations for encoder, used for pose estimation - how trusted encoder measurements are
    private static final Vector<N3> encoderStateDeviations = VecBuilder.fill(1.0, 1.0, 1.0);

    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private final PIDController rotationOverrideController = new PIDController(0.5, 0, 0);

    private boolean rotationOverride = false;

    private final PIDController translationOverrideController = new PIDController(0, 0,0);

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

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
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
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);

        Logger.recordOutput("SwerveStates/AutoLock", rotationOverride);
    }


    /**
     * Drives the robot with a specified (robot-relative) velocity
     */
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
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
