// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RgbLEDs;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.lib.Controller;
import frc.lib.FlightControl;

import java.util.PrimitiveIterator;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.runShooter;
import frc.robot.commands.runShooterHoodBack;
import frc.robot.commands.runShooterHoodForward;
import frc.robot.commands.runVariableShooter;
import frc.robot.constants.OperatorConstants;

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

  private final Controller controller = new Controller(5);
  private final Controller teoController = new Controller(2);
  private final Joystick joystick = new Joystick(1);
  private final JoystickButton b1 = new JoystickButton(joystick, 1);
  private final JoystickButton b2 = new JoystickButton(joystick, 2);
  private final JoystickButton b3 = new JoystickButton(joystick, 3);
  private final JoystickButton b4 = new JoystickButton(joystick, 4);

  private final Limelight cam = new Limelight();

  // private final SwerveDrive sd = new SwerveDrive(cam);

  private final FlightControl flightcont = new FlightControl(3);

  private final RgbLEDs rgbLeds = new RgbLEDs();

  private final Shooter shooter = new Shooter();

  private final runShooter runshooter = new runShooter(shooter);

  private final runShooterHoodForward runshooterhoodforward = new runShooterHoodForward(shooter);
  private final runShooterHoodBack runshooterhoodback = new runShooterHoodBack(shooter);

  private final runVariableShooter runshooterV = new runVariableShooter(shooter);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    CameraServer.startAutomaticCapture();

    // Shooter.setDefaultCommand(new RunCommand(()-> Shooter.RunFeeder(), Shooter));
    configureControllerBindings();
    // configureJoystickBindings();
    configureOperatorBindings();
    // configureSubsystemCommands();
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
    b1.whileTrue(runshooterV);
    b2.onTrue(runshooter);

    //b1.whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //b2.whileTrue(runshooterhoodback);
    //b3.whileTrue(runshooterhoodforward);
    //b2.whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //b3.whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //b4.whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    /*
     * cam.setDefaultCommand(
     * new RunCommand(() -> cam.updatesd(), cam)
     * );
     */

    // sd.setDefaultCommand(
    // new RunCommand(() -> sd.drive(
    // -MathUtil.applyDeadband(teoController.getLeftX(),
    // OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(teoController.getLeftY(),
    // OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(teoController.getRightX(),
    // OperatorConstants.DEADBAND),
    // false,
    // true,
    // true)
    // ,sd)
    // );

    // teoController.getRightTrigger().whileTrue(new RunCommand(() -> sd.drive(
    // -MathUtil.applyDeadband(teoController.getLeftX(),
    // OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(teoController.getLeftY(),
    // OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(teoController.getRightX(),
    // OperatorConstants.DEADBAND),
    // true,
    // true,
    // true)
    // ,sd));

    // teoController.getLeftTrigger().whileTrue(new RunCommand(() -> sd.drive(
    // -MathUtil.applyDeadband(teoController.getLeftY() * -1,
    // OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(teoController.getLeftX(),
    // OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(teoController.getRightX(),
    // OperatorConstants.DEADBAND),
    // true,
    // true,
    // true)
    // ,sd));

  }

  public void configureJoystickBindings() {

    // sd.setDefaultCommand(
    // new RunCommand(() -> sd.drive(
    // -MathUtil.applyDeadband(flightcont.getJoyX(), OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND),
    // false,
    // true,
    // true)
    // ,sd)
    // );

    //flightcont.getButton1().whileTrue(runshooter);
    //flightcont.getButton1().whileTrue(new RunCommand(() -> shooter.runVariableMotorFeeder(), shooter));

    // flightcont.getButton1().whileTrue(new RunCommand(() -> sd.drive(
    // -MathUtil.applyDeadband(flightcont.getJoyX(), OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND),
    // -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND),
    // true,
    // true,
    // true)
    // ,sd));

    // flightcont.getButton2().whileTrue(new RunCommand(() -> sd.zeroHeading(),
    // sd));
  }

  public void configureOperatorBindings() {

    // example commands
    // controller.getLeftBumper().whileTrue();
    // controller.getRightBumper().onTrue();
    controller.getY().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.getA().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.getX().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.getB().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  private void configureSwerveDriveCommands() {
    // teoController.getDown()
    // .whileTrue(
    // new RunCommand(
    // () -> sd.zeroHeading(),
    // sd
    // )
    // );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
