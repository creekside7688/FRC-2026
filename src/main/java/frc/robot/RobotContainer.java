// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.google.flatbuffers.Constants;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Controller;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIONavX;
import frc.robot.subsystems.drivebase.GyroIOSim;
import frc.robot.subsystems.drivebase.SwerveDrive;
import frc.robot.subsystems.drivebase.module.ModuleIO;
import frc.robot.subsystems.drivebase.module.ModuleIOMapleSim;
import frc.robot.subsystems.drivebase.module.ModuleIOSparkMax;
import frc.robot.subsystems.drivebase.module.SwerveModule;
import frc.robot.subsystems.drivebase.vision.VisionIOLimelight;
import frc.robot.subsystems.drivebase.vision.VisionIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        private final Controller operatorController = new Controller(1);
        private final Controller driveController = new Controller(2);

        private final VisionIOLimelight cam;

        private final SwerveDrive sd;

        private final DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
                        .withGyro(COTS.ofNav2X())
                        .withTrackLengthTrackWidth(Units.Meters.of(DriveConstants.TRACK_WIDTH),
                                        Units.Meters.of(DriveConstants.TRACK_WIDTH))
                        .withRobotMass(Units.Kilograms.of(50))
                        .withSwerveModule(new SwerveModuleSimulationConfig(
                                        DCMotor.getNEO(1),
                                        DCMotor.getNeo550(1),
                                        ModuleConstants.DRIVE_MOTOR_REDUCTION,
                                        ModuleConstants.TURN_MOTOR_REDUCTION,
                                        Units.Volts.of(0.1),
                                        Units.Volts.of(0.1),
                                        Units.Meters.of(ModuleConstants.WHEEL_RADIUS_METRES),
                                        Units.KilogramSquareMeters.of(0.02),
                                        COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof))
                        .withBumperSize(Units.Inches.of(30), Units.Inches.of(30));

        private final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(simulationConfig,
                        new Pose2d(3, 3, new Rotation2d()));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                ModuleIO fl, fr, bl, br;
                GyroIO gyro;
                if (RobotBase.isReal()) {

                        /*
                         * REAL HARDWARE IO
                         */

                        fl = new ModuleIOSparkMax(DriveConstants.FL_DRIVE_MOTOR, DriveConstants.FL_TURN_MOTOR,
                                        Rotation2d.fromRadians(DriveConstants.FL_OFFSET));
                        fr = new ModuleIOSparkMax(DriveConstants.FR_DRIVE_MOTOR, DriveConstants.FR_TURN_MOTOR,
                                        Rotation2d.fromRadians(DriveConstants.FR_OFFSET));
                        bl = new ModuleIOSparkMax(DriveConstants.BL_DRIVE_MOTOR, DriveConstants.BL_TURN_MOTOR,
                                        Rotation2d.fromRadians(DriveConstants.BL_OFFSET));
                        br = new ModuleIOSparkMax(DriveConstants.BR_DRIVE_MOTOR, DriveConstants.BR_TURN_MOTOR,
                                        Rotation2d.fromRadians(DriveConstants.BR_OFFSET));
                        gyro = new GyroIONavX(NavXComType.kUSB1);
                        cam = new VisionIOLimelight(VisionConstants.ROBOT_TO_CAM);
                } else {

                        /*
                         * MAPLESIM SIMULATION IO
                         */

                        fl = new ModuleIOMapleSim(driveSimulation.getModules()[0]);
                        fr = new ModuleIOMapleSim(driveSimulation.getModules()[1]);
                        bl = new ModuleIOMapleSim(driveSimulation.getModules()[2]);
                        br = new ModuleIOMapleSim(driveSimulation.getModules()[3]);

                        gyro = new GyroIOSim(driveSimulation.getGyroSimulation());
                        cam = new VisionIOSim(driveSimulation::getSimulatedDriveTrainPose,
                                        VisionConstants.ROBOT_TO_CAM);

                        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                }

                sd = new SwerveDrive(cam, gyro, fl, fr, bl, br);

                configureControllerBindings();
                configureOperatorBindings();

                configureSwerveDriveCommands();

                // rgbLeds.RgbSolidRed();
        }

        /**
         * Use whis method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * 
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureControllerBindings() {

                sd.setDefaultCommand(
                                new RunCommand(() -> sd.drive(
                                                -MathUtil.applyDeadband(driveController.getLeftX(),
                                                                OperatorConstants.DEADBAND),
                                                -MathUtil.applyDeadband(driveController.getLeftY(),
                                                                OperatorConstants.DEADBAND),
                                                -MathUtil.applyDeadband(driveController.getRightX(),
                                                                OperatorConstants.DEADBAND),
                                                false,
                                                true,
                                                true), sd));

                driveController.getRightTrigger().whileTrue(new RunCommand(() -> sd.drive(
                                -MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(driveController.getRightX(), OperatorConstants.DEADBAND),
                                true,
                                true,
                                true), sd));

                driveController.getLeftTrigger().whileTrue(new RunCommand(() -> sd.drive(
                                -MathUtil.applyDeadband(driveController.getLeftY() * -1, OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(driveController.getRightX(), OperatorConstants.DEADBAND),
                                true,
                                true,
                                true), sd));

        }

        public void configureOperatorBindings() {

        }

        private void configureSwerveDriveCommands() {
                driveController.getDown()
                                .whileTrue(
                                                new RunCommand(
                                                                () -> sd.zeroHeading(),
                                                                sd));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return sd.followPath(new Pose2d(10, 6, new Rotation2d()));
        }

        public void updateSimulation() {
                SimulatedArena.getInstance().simulationPeriodic();
                Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Fuel",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

                Logger.recordOutput("FieldSimulation/SwerveModuleStates",
                                DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                                                driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative()));
                Logger.recordOutput("FieldSimulation/SwerveModuleSpeeds",
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative());

        }
}
