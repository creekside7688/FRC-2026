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

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
  private SparkMaxConfig config1;
  private SparkMaxConfig config2;
  private SparkMaxConfig configHood;
  
  private final SysIdRoutine routine;


  private final SparkMax shootMotor1;
  private final SparkMax shootMotor2 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID2, MotorType.kBrushless);
  
  private final SparkMax hoodMotor;
  private final AbsoluteEncoder hoodMotorEncoder;
  private final AbsoluteEncoder shootMotor1Encoder; 

  private final TalonSRX feedControllerSrx = new TalonSRX(ShooterConstants.FEED_MOTOR_SRX_ID); //I actually dunno this ID

  
  
  private final SparkClosedLoopController sm1_Controller; 
  private final SparkClosedLoopController hood_Controller;
  
  
  @SuppressWarnings("removal")
  public shooter() {
    
    this.hoodMotor = new SparkMax(ShooterConstants.BALL_HOOD_MOTOR, MotorType.kBrushless);
    this.shootMotor1 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID1, MotorType.kBrushless);

    this.hoodMotorEncoder = this.hoodMotor.getAbsoluteEncoder();
    this.shootMotor1Encoder = this.hoodMotor.getAbsoluteEncoder();

    sm1_Controller = this.shootMotor1.getClosedLoopController();
    hood_Controller = this.hoodMotor.getClosedLoopController();

    config1 = new SparkMaxConfig();
    config2 = new SparkMaxConfig();
    configHood = new SparkMaxConfig();
    


    double kP = 0.0001; //??
    double kF = 1.0 / 5676.0;
    config1
        .closedLoop
            .p(ShooterConstants.SHOOTER_P)
            .i(ShooterConstants.SHOOTER_I)
            .d(ShooterConstants.SHOOTER_D)

            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .feedForward
                .kV(ShooterConstants.SHOOTER_KV)
                .kA(ShooterConstants.SHOOTER_KA)
                .kS(ShooterConstants.SHOOTER_KS);

    
    config2
        .inverted(true)
        .follow(shootMotor1);



    this.shootMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
    this.routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(shootMotor1::setVoltage, null, this)
    );

  }

  public void SetRPM(int rpm) {
    sm1_Controller.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void RunIdle() {
    SetRPM(ShooterConstants.IDLE_RPM);
  }

  public void setShooterMotor1Voltage(double volts) {
    shootMotor1.setVoltage(volts);
  }


  public void RunFeeder() {
    feedControllerSrx.set(ControlMode.PercentOutput, ShooterConstants.RUN_FEEDER_OUTPUT);
  }
  


  public void setHoodMotorPosition(int setPoint) {
     hood_Controller.setSetpoint(setPoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
