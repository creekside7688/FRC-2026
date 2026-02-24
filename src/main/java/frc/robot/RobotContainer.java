// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Controller;
import frc.lib.FlightControl;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterHood;
import frc.robot.subsystems.Shooter.testShootingLookup;
import frc.robot.subsystems.Shooter.testVDS;
import frc.robot.subsystems.Shooter.testVariableHoodPosition;
import frc.robot.subsystems.Shooter.testVariableRPMFlywheel;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIONavX;
import frc.robot.subsystems.drivebase.GyroIOSim;
import frc.robot.subsystems.drivebase.SwerveDrive;
import frc.robot.subsystems.drivebase.module.ModuleIO;
import frc.robot.subsystems.drivebase.module.ModuleIOMapleSim;
import frc.robot.subsystems.drivebase.module.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeBackCommand;
import frc.robot.subsystems.intake.IntakeForwardCommand;
import frc.robot.subsystems.intake.IntakeRollerBackCommand;
import frc.robot.subsystems.intake.IntakeRollerStopCommand;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

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

    private final Intake intake = new Intake();
    private final IntakeBackCommand intakeBackCommand = new IntakeBackCommand(intake);
    private final IntakeForwardCommand intakeForwardCommand = new IntakeForwardCommand(intake);
    private final IntakeRollerBackCommand intakeRollerBackCommand = new IntakeRollerBackCommand(intake);
    private final IntakeRollerStopCommand intakeRollerForwardCommand = new IntakeRollerStopCommand(intake);
    private final Command intakeRollerStop = new Command() {};

    private final Controller operatorController = new Controller(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final Controller driveController = new Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);

    private final FlightControl joystick = new FlightControl(0);
    private final Shooter shooter = new Shooter();
    private final ShooterHood shooterhood = new ShooterHood();
    private final Spindexer spindexer = new Spindexer();
    private final testVariableRPMFlywheel tesShooterVariable = new testVariableRPMFlywheel(shooter);

    Command ShootRPM;
    Command ShootAngle; // PLacebo values I dunno
    Command ShootFeederRun = shooter.runShooterFeeder();
    Command ShootFeederStop = shooter.stopShooterFeeder();
    Command spindexerRun = spindexer.runSpindexerMotor();
    Command spindexerStop = spindexer.stopSpindexerMotor();

    private final testVariableRPMFlywheel testshooter =
            new testVariableRPMFlywheel(shooter); // test that feeder + flywheel work

    private final testVariableHoodPosition testhood =
            new testVariableHoodPosition(shooterhood); // test that hood + indexer works

    private final testVDS testvds =
            new testVDS(shooter, shooterhood); // for when we get values for lookup table through testing

    private final testShootingLookup testlookup =
            new testShootingLookup(shooter); // for when we do lookup table testing

    @SuppressWarnings("unused")
    private final Vision camSystem;

    private final SwerveDrive sd;

    private final DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofNav2X())
            .withTrackLengthTrackWidth(
                    Units.Meters.of(DrivebaseConstants.TRACK_WIDTH), Units.Meters.of(DrivebaseConstants.TRACK_WIDTH))
            .withRobotMass(Units.Pounds.of(DrivebaseConstants.ROBOT_MASS_KG))
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getNEO(1),
                    DCMotor.getNeo550(1),
                    ModuleConstants.DRIVE_MOTOR_REDUCTION,
                    ModuleConstants.TURN_MOTOR_REDUCTION,
                    Units.Volts.of(0.1),
                    Units.Volts.of(0.1),
                    Units.Meters.of(ModuleConstants.WHEEL_RADIUS_METRES),
                    Units.KilogramSquareMeters.of(0.02),
                    1.2))
            .withBumperSize(Units.Inches.of(33), Units.Inches.of(33));

    private final SwerveDriveSimulation driveSimulation =
            new SwerveDriveSimulation(simulationConfig, new Pose2d(3, 3, new Rotation2d()));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        ModuleIO fl, fr, bl, br;
        GyroIO gyro;
        VisionIO camIO1, camIO2;
        switch (RobotState.CURRENT_MODE) {
            case REAL:
                fl = new ModuleIOSparkMax(
                        DrivebaseConstants.FL_DRIVE_MOTOR,
                        DrivebaseConstants.FL_TURN_MOTOR,
                        Rotation2d.fromRadians(DrivebaseConstants.FL_OFFSET));
                fr = new ModuleIOSparkMax(
                        DrivebaseConstants.FR_DRIVE_MOTOR,
                        DrivebaseConstants.FR_TURN_MOTOR,
                        Rotation2d.fromRadians(DrivebaseConstants.FR_OFFSET));
                bl = new ModuleIOSparkMax(
                        DrivebaseConstants.BL_DRIVE_MOTOR,
                        DrivebaseConstants.BL_TURN_MOTOR,
                        Rotation2d.fromRadians(DrivebaseConstants.BL_OFFSET));
                br = new ModuleIOSparkMax(
                        DrivebaseConstants.BR_DRIVE_MOTOR,
                        DrivebaseConstants.BR_TURN_MOTOR,
                        Rotation2d.fromRadians(DrivebaseConstants.BR_OFFSET));
                gyro = new GyroIONavX(NavXComType.kUSB2);
                camIO1 = new VisionIOPhotonVision("SWERVE_CAM", VisionConstants.ROBOT_TO_SWERVE_CAM_TRANSFORM);
                camIO2 = new VisionIOPhotonVision("HOPPER_CAM", VisionConstants.ROBOT_TO_HOPPER_CAM_TRANSFORM);
                break;

            case SIM:
                fl = new ModuleIOMapleSim(driveSimulation.getModules()[0]);
                fr = new ModuleIOMapleSim(driveSimulation.getModules()[1]);
                bl = new ModuleIOMapleSim(driveSimulation.getModules()[2]);
                br = new ModuleIOMapleSim(driveSimulation.getModules()[3]);

                gyro = new GyroIOSim(driveSimulation.getGyroSimulation());
                camIO1 = new VisionIOPhotonVisionSim(
                        "SWERVE_CAM",
                        VisionConstants.ROBOT_TO_SWERVE_CAM_TRANSFORM,
                        driveSimulation::getSimulatedDriveTrainPose);
                camIO2 = new VisionIOPhotonVisionSim(
                        "HOPPER_CAM",
                        VisionConstants.ROBOT_TO_HOPPER_CAM_TRANSFORM,
                        driveSimulation::getSimulatedDriveTrainPose);

                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                break;
            default: // REPLAY MODE - disable IO implementations, inputs provided by replay file
                fl = new ModuleIO() {};
                fr = new ModuleIO() {};
                bl = new ModuleIO() {};
                br = new ModuleIO() {};

                gyro = new GyroIO() {};
                camIO1 = new VisionIO() {};
                camIO2 = new VisionIO() {};
        }

        sd = new SwerveDrive(gyro, fl, fr, bl, br);
        camSystem = new Vision(sd, camIO1, camIO2);

        ShootRPM = shooter.setShooterRPM(sd::getPose);
        ShootAngle = shooterhood.setShooterAngle(sd::getPose);

        configureDriveBindings();
        configureOperatorBindings();
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
    private void configureDriveBindings() {

        // sd.setDefaultCommand(TeleopDrive.joystickDrive(
        //         sd,
        //         () -> -driveController.getLeftY(),
        //         () -> -driveController.getLeftX(),
        //         () -> -driveController.getRightX()));

        // driveController
        //         .getA()
        //         .whileTrue(TeleopDrive.joystickDriveWithRotationalOverride(
        //                 sd,
        //                 () -> -driveController.getLeftY(),
        //                 () -> -driveController.getLeftX(),
        //                 () -> SwerveUtils.lookAtPoint(
        //                         sd.getPose().getTranslation(), GameConstants.HUB_BLUE.toTranslation2d())));

        // driveController
        //         .getB()
        //         .whileTrue(TeleopDrive.joystickDriveWithTrenchAlign(sd, () -> -driveController.getLeftY()));
        // driveController.getDown().whileTrue(new RunCommand(() -> sd.zeroHeading(), sd));
    }

    public void configureOperatorBindings() {

        joystick.getButton1().whileTrue(testhood);
        joystick.getButton3().whileTrue(intakeBackCommand);
        joystick.getButton4().whileTrue(intakeForwardCommand);
        joystick.getButton5().whileTrue(intakeRollerBackCommand);
        joystick.getButton6().whileTrue(intakeRollerForwardCommand);
        // driveController.getA().whileTrue(testhood);
        driveController.getLeftBumper().whileTrue(intakeBackCommand);
        driveController.getRightBumper().whileTrue(intakeForwardCommand);
        driveController.getLeftTrigger().whileTrue(intakeRollerBackCommand);
        driveController.getRightTrigger().whileTrue(intakeRollerForwardCommand);

        Command holdxuntilflywheel = ShootRPM.alongWith(
                ShootAngle,
                ShootFeederRun.onlyIf(
                        () -> shooter.checkShooterRPMTolerance() && shooterhood.checkShooterPositionTolerance()),
                spindexerRun.onlyIf(
                        () -> shooter.checkShooterRPMTolerance() && shooterhood.checkShooterPositionTolerance()));

        operatorController.getX().whileTrue(holdxuntilflywheel);

        Command DeployRunIntake = intakeForwardCommand.andThen(intakeRollerForwardCommand);
        driveController.getB().whileTrue(DeployRunIntake);

        Command StopStowIntake = intakeRollerStop.andThen(intakeRollerBackCommand);
        intake.setDefaultCommand(StopStowIntake);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new Command() {};
    }

    // ONLY RUNS iN SIMULATION
    public void updateSimulation() {
        // General Simulation
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());

        // Game-Specific Simulation
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
