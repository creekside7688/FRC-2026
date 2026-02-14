// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */
  private SparkMaxConfig config1;
  private SparkMaxConfig config2;
  private SparkMaxConfig configHood;

  private ShuffleboardTab tab = Shuffleboard.getTab("shooter");
  private GenericEntry voltage = tab.add("shooterVoltage", 0).getEntry();

  private GenericEntry feederSpeed = tab.add("feederSpeed", 0).getEntry();

  private GenericEntry hoodVoltage = tab.add("hoodVoltage", 0).getEntry();

  private GenericEntry hoodPos = tab.add("hoodPos", 60).getEntry();

  private GenericEntry shootRPM = tab.add("shootRPM", 0).getEntry();

  


  private final SparkMax shootMotor1 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID1, MotorType.kBrushless);
  private final SparkMax shootMotor2 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID2, MotorType.kBrushless);

  private final SparkMax hoodMotor;
  private final AbsoluteEncoder hoodMotorEncoder;
  private final AbsoluteEncoder shootMotor1Encoder;

  private final TalonSRX feedControllerSrx = new TalonSRX(ShooterConstants.FEED_MOTOR_SRX_ID); // I actually dunno this
                                                                                               // ID

  private final SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(shootMotor1::setVoltage, null, this));

  private final SparkClosedLoopController sm1_Controller;
  private final SparkClosedLoopController hood_Controller;

  public Shooter() {
    SmartDashboard.putBoolean("t", false);

    this.hoodMotor = new SparkMax(ShooterConstants.BALL_HOOD_MOTOR, MotorType.kBrushless);

    this.hoodMotorEncoder = this.hoodMotor.getAbsoluteEncoder();
    this.shootMotor1Encoder = this.hoodMotor.getAbsoluteEncoder();

    sm1_Controller = this.shootMotor1.getClosedLoopController();
    hood_Controller = this.hoodMotor.getClosedLoopController();

    config1 = new SparkMaxConfig();
    config2 = new SparkMaxConfig();
    configHood = new SparkMaxConfig();
    
    config1.encoder.uvwAverageDepth(2);
    config2.encoder.uvwAverageDepth(2);

    config1.encoder.uvwMeasurementPeriod(16);
    config2.encoder.uvwMeasurementPeriod(16);

 
    config1.closedLoop
        .p(ShooterConstants.SHOOTER_P)
        .i(ShooterConstants.SHOOTER_I)
        .d(ShooterConstants.SHOOTER_D)

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward
        .kV(ShooterConstants.SHOOTER_KV)
        .kA(ShooterConstants.SHOOTER_KA)
        .kS(ShooterConstants.SHOOTER_KS);

    config1.encoder.velocityConversionFactor(1);
    config1.signals.primaryEncoderVelocityAlwaysOn(true);
    config1.signals.primaryEncoderPositionAlwaysOn(true);

    config1.smartCurrentLimit(50);
    config2.smartCurrentLimit(50);

    config1.idleMode(IdleMode.kCoast);
    config2.idleMode(IdleMode.kCoast);

    config2.follow(shootMotor1, true);

    configHood.closedLoop
      .p(0.1)
      .i(0)
      .d(0);

    configHood.encoder
            .positionConversionFactor(ShooterConstants.ANGLECHANGE_PER_ROTATION);

        // Soft Limits: Prevent the hood from slamming into the frame
    configHood.softLimit
        .forwardSoftLimit(75) // Max angle
        .reverseSoftLimit(45) // Min angle
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    configHood.idleMode(IdleMode.kBrake);



    hoodMotor.getEncoder().setPosition(60);

    this.shootMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodMotor.configure(configHood, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }





  public void SetRPM(int rpm) {
    sm1_Controller.setSetpoint(rpm, ControlType.kVelocity);

  }

  public void SetVariableRPM() {
    int rpm = (int) shootRPM.getDouble(0);
    SetRPM(rpm);
  }

  public void RunIdle() {
    SetRPM(ShooterConstants.IDLE_RPM);
  }

  public void setShooterMotor1Voltage(double volts) {
    shootMotor1.setVoltage(volts);
  }



  public void runVariableVoltage() {
    SmartDashboard.putBoolean("t", true);
    double retrievedVoltage = voltage.getDouble(0);
    shootMotor1.setVoltage(retrievedVoltage);
    SmartDashboard.putNumber("set voltage", retrievedVoltage);
  }

  public void RunFeeder() {
    feedControllerSrx.set(ControlMode.PercentOutput, ShooterConstants.RUN_FEEDER_OUTPUT);
  }

  public void stopSystem() {
    stopFeeder();
    shootMotor1.setVoltage(0);
  }





  public void resetHoodEncoder(double currentActualAngle) {
    hoodMotor.getEncoder().setPosition(currentActualAngle);
  }

  public void setHoodMotorPosition(int setPoint) {
    hood_Controller.setSetpoint(setPoint, ControlType.kPosition);
  }

  public void setVariableMotorPosition() {
    int setPoint = (int) (hoodPos.getDouble(0));
    setHoodMotorPosition(setPoint);
  }
  
  public void setHoodMotorVoltage(double volts) {
    hoodMotor.setVoltage(volts);
  }

  public void setVariableHMVoltage(boolean inverted) {
    double hoodMVolts = hoodVoltage.getDouble(0);
    if (inverted) hoodMVolts *= -1;
    setHoodMotorVoltage(hoodMVolts);
  }

  /* 

  */





  public void stopFeeder() {
    feedControllerSrx.set(ControlMode.PercentOutput, 0);
  }

  public void runVariableMotorFeeder() {
    double retriedFS = feederSpeed.getDouble(0);
    SmartDashboard.putNumber("m", retriedFS);
    feedControllerSrx.set(ControlMode.PercentOutput, retriedFS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run;
    SmartDashboard.putNumber("Shooter 1 Pos", shootMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter 1 Velocity",shootMotor1.getEncoder().getVelocity());

    SmartDashboard.putNumber("Shooter 2 Pos",shootMotor2.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter 3 Velocity",shootMotor2.getEncoder().getVelocity());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */

}
