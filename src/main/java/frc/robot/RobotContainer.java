// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.constants.VisionConstants.ROBOT_TO_HOPPER_CAM_TRANSFORM;
import static frc.robot.constants.VisionConstants.ROBOT_TO_SWERVE_CAM_TRANSFORM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Controller;
import frc.lib.FlightControl;
import frc.lib.SwerveUtils;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PositionPIDCommand;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.ClimberPost;
import frc.robot.subsystems.climber.commands.ClimberPre;
import frc.robot.subsystems.climber.commands.ClimberZero;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIONavX;
import frc.robot.subsystems.drivebase.GyroIOSim;
import frc.robot.subsystems.drivebase.SwerveDrive;
import frc.robot.subsystems.drivebase.module.ModuleIO;
import frc.robot.subsystems.drivebase.module.ModuleIOMapleSim;
import frc.robot.subsystems.drivebase.module.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBackCommand;
import frc.robot.subsystems.intake.commands.IntakeFixAngleBack;
import frc.robot.subsystems.intake.commands.IntakeFixAngleForward;
import frc.robot.subsystems.intake.commands.IntakeForwardCommand;
import frc.robot.subsystems.intake.commands.IntakeRollerForwardCommand;
import frc.robot.subsystems.intake.commands.IntakeStopCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.ShooterHood;
import frc.robot.subsystems.shooter.commands.RunShooter;
import frc.robot.subsystems.shooter.commands.TestShootingLookup;
// import frc.robot.subsystems.shooter.commands.TestVDS;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.commands.RunSpindexer;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
    public final IntakeBackCommand intakeBackCommand = new IntakeBackCommand(intake);
    public final IntakeForwardCommand intakeForwardCommand = new IntakeForwardCommand(intake);
    public final IntakeRollerForwardCommand intakeRollerForwardCommand = new IntakeRollerForwardCommand(intake);
    private final IntakeFixAngleForward intakeFixAngleForwardCommand = new IntakeFixAngleForward(intake);
    private final IntakeFixAngleBack intakeFixAngleBackCommand = new IntakeFixAngleBack(intake);
    private final IntakeStopCommand intakeStopCommand = new IntakeStopCommand(intake);
    private final IntakeRollerForwardCommand rollerAgitate = new IntakeRollerForwardCommand(intake);

    private final Controller operatorController = new Controller(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final Controller driveController = new Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);

    private final FlightControl joystick = new FlightControl(0);

    private final Shooter shooter = new Shooter();
    private final ShooterHood shooterhood = new ShooterHood();
    private final ShooterFeeder feeder = new ShooterFeeder();
    private final Spindexer spindexer = new Spindexer();

    private final Climber climber = new Climber();
    private final ClimberPre climberPre = new ClimberPre(climber);
    private final ClimberPost climberPost = new ClimberPost(climber);
    private final ClimberZero climberZero = new ClimberZero(climber);

    private final RunSpindexer runspindexer = new RunSpindexer(spindexer);

    private final TestShootingLookup testlookup =
            new TestShootingLookup(shooter, feeder, shooterhood, spindexer); // for

    //     private final TestVDS testvds = new TestVDS(shooter, shooterhood, feeder, spindexer);

    private final Command autoClimbLeft;
    private final Command autoClimbRight;
    // when
    // we
    // do
    // lookup
    // table
    // testing

    private final RunShooter runShooter;

    private final LoggedDashboardChooser<Command> autoChooser;

    // private final LEDLights ledLights = new LEDLights();

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
                gyro = new GyroIONavX();
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

        runShooter = new RunShooter(shooter, sd, feeder, shooterhood, spindexer);

        autoClimbRight = PositionPIDCommand.generateCommand(sd, false, 3);

        autoClimbLeft = PositionPIDCommand.generateCommand(sd, true, 3);
        // ShootRPM = shooter.setShooterRPM(sd::getPose);
        // ShootAngle = shooterhood.setShooterAngle(sd::getPose);

        NamedCommands.registerCommand(
                "Align To Hub",
                new DriveCommands.AutonomousHubAlign(
                        sd,
                        () -> SwerveUtils.lookAtPoint(
                                sd.getPose()
                                        .plus(ShooterConstants.ROBOT_TO_SHOOTER)
                                        .getTranslation(),
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                        ? GameConstants.HUB_RED
                                        : GameConstants.HUB_BLUE)));
        NamedCommands.registerCommand("Shoot", runShooter);
        NamedCommands.registerCommand("Deploy Intake", intakeForwardCommand);
        NamedCommands.registerCommand("Stow Intake", intakeBackCommand);
        NamedCommands.registerCommand("Run Intake Rollers", intakeRollerForwardCommand);
        NamedCommands.registerCommand("Climb On Left", autoClimbRight);
        NamedCommands.registerCommand("Climb On Right", autoClimbLeft);
        NamedCommands.registerCommand("Climb Motors Run", climberPost);

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

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

        sd.setDefaultCommand(DriveCommands.joystickDrive(
                sd,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX()));

        driveController
                .getRightTrigger()
                .whileTrue(DriveCommands.joystickDriveWithRotationalOverride(
                        sd,
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        () -> SwerveUtils.lookAtPoint(
                                sd.getPose()
                                        .plus(ShooterConstants.ROBOT_TO_SHOOTER)
                                        .getTranslation(),
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                        ? GameConstants.HUB_RED
                                        : GameConstants.HUB_BLUE)));

        // driveController
        //         .getRightBumper()
        //         .whileTrue(DriveCommands.joystickDriveWithTrenchAlign(sd, () -> -driveController.getLeftY()));
        driveController.getRightBumper().whileTrue(autoClimbRight);
        driveController.getLeftBumper().whileTrue(autoClimbLeft);
    }

    public void configureOperatorBindings() {
        operatorController.getRightBumper().whileTrue(intakeForwardCommand.andThen(intakeRollerForwardCommand));
        operatorController.getRightBumper().whileFalse(intakeFixAngleBackCommand);
        operatorController.getLeftBumper().whileTrue(rollerAgitate);

        operatorController.getX().whileTrue(runShooter);

        operatorController.getLeftTrigger().whileTrue(climberZero);
        operatorController.getRightTrigger().whileTrue(climberPost);
        // joystick.getButton1().whileTrue(testvds);
        /*
        joystick.getButton3().whileTrue(intakeForwardCommand.andThen(intakeRollerForwardCommand));
        joystick.getButton4().whileFalse(intakeFixAngleBackCommand);
        joystick.getButton5().whileTrue(intakeStopCommand);
        */
        // joystick.getButton6().whileTrue(climberPre);
        // joystick.getButton3().whileTrue(climberPost);
        // joystick.getButton5().whileTrue(climberZero);

        shooterhood.setDefaultCommand(shooterhood.runOnce(() -> shooterhood.setHoodPosition(75)));

        // joystick.getButton1().whileTrue(runShooter);
        // joystick.getButton2().whileTrue(intakeFixAngleBackCommand);
        // Command shooterRunSequential = ShootRPM.alongWith(
        // ShootAngle,
        // ShootFeederRun.onlyIf(
        // () -> shooter.checkShooterRPMTolerance() &&
        // shooterhood.checkShooterPositionTolerance()),
        // spindexerRun.onlyIf(
        // () -> shooter.checkShooterRPMTolerance() &&
        // shooterhood.checkShooterPositionTolerance()));

        // joystick.getButton3().whileTrue(Commands.run(() -> shooter.SetRPM(4000), shooter));

        // operatorController.getX().whileTrue(shooterRunSequential);

        // joystick.getButton5().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.getButton3().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // joystick.getButton6().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.getButton4().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.get();
    }

    // ONLY RUNS iN SIMULATION
    public void updateSimulation() {
        // General Simulation
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());

        // Game-Specific Simulation
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

        Pose3d robotPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("SwerveOffset", robotPose.plus(ROBOT_TO_SWERVE_CAM_TRANSFORM));
        Logger.recordOutput("HopperOffset", robotPose.plus(ROBOT_TO_HOPPER_CAM_TRANSFORM));
        Logger.recordOutput(
                "ShooterOffset",
                new Pose3d(driveSimulation.getSimulatedDriveTrainPose().plus(ShooterConstants.ROBOT_TO_SHOOTER))
                        .plus(new Transform3d(
                                new Translation3d(Inches.of(0), Inches.of(0), Inches.of(17)), Rotation3d.kZero)));
        Logger.recordOutput(
                "test dist",
                driveSimulation
                        .getSimulatedDriveTrainPose()
                        .plus(ShooterConstants.ROBOT_TO_SHOOTER)
                        .getTranslation()
                        .getDistance((GameConstants.HUB_RED)));
    }
}
