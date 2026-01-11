// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RgbLEDs;
import frc.robot.subsystems.SwerveDrive;
import frc.lib.Controller;
import frc.lib.FlightControl;

import java.util.PrimitiveIterator;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive sd = new SwerveDrive();

  private final Controller controller = new Controller(1 );
  private final Controller teoController = new Controller(2);

  private final Limelight cam =  new Limelight();

  private final FlightControl flightcont = new FlightControl(3);

  private final RgbLEDs rgbLeds = new RgbLEDs();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    CameraServer.startAutomaticCapture();
    configureControllerBindings();
    //configureJoystickBindings();
    configureOperatorBindings();
    //configureSubsystemCommands();
    configureSwerveDriveCommands();

    //rgbLeds.RgbSolidRed();
  }


  /**
   * Use whis method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * 
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureControllerBindings() {

  

    cam.setDefaultCommand(
      new RunCommand(() -> cam.updatesd(), cam)
    );

    
    sd.setDefaultCommand(
      new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(teoController.getLeftX(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getLeftY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getRightX(), OperatorConstants.DEADBAND), 
        false, 
        true, 
        true)
      ,sd)
    );




    teoController.getRightTrigger().whileTrue(new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(teoController.getLeftX(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getLeftY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getRightX(), OperatorConstants.DEADBAND), 
        true, 
        true, 
        true)
      ,sd));


      teoController.getLeftTrigger().whileTrue(new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(teoController.getLeftY() * -1, OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getLeftX(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(teoController.getRightX(), OperatorConstants.DEADBAND), 
        true, 
        true, 
        true)
      ,sd));


    
  }

  public void configureJoystickBindings(){
     
    sd.setDefaultCommand(
      new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(flightcont.getJoyX(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND), 
        false, 
        true, 
        true)
      ,sd)
    );



    flightcont.getButton1().whileTrue(new RunCommand(() -> sd.drive(
        -MathUtil.applyDeadband(flightcont.getJoyX(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getJoyY(), OperatorConstants.DEADBAND), 
        -MathUtil.applyDeadband(flightcont.getTwist(), OperatorConstants.DEADBAND), 
        true, 
        true, 
        true)
      ,sd));

    flightcont.getButton2().whileTrue(new RunCommand(() -> sd.zeroHeading(), sd));
  }


  public void configureOperatorBindings(){
   
    
    //example commands
    controller.getLeftBumper().whileTrue();
    controller.getRightBumper().onTrue();
    

  }



  private void configureSwerveDriveCommands() {
    teoController.getDown()
        .whileTrue(
            new RunCommand(
                () -> sd.zeroHeading(),
                sd
            )
        );


}

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return ab;
  }
}
