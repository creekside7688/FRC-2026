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

import com.ctre.phoenix.motorcontrol.ControlMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */

  
  
  private final SparkMax shootMotor1 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID1, MotorType.kBrushless);
  private final SparkMax shootMotor2 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID2, MotorType.kBrushless);
  
  private final SparkMax hoodMotor;
  private final AbsoluteEncoder hoodMotorEncoder;

  private final TalonSRX feedControllerSrx = new TalonSRX(0); //I actually dunno this ID

  
  
  private final SparkClosedLoopController sm1_Controller = shootMotor1.getClosedLoopController();

  //private final SparkClosedLoopController hood_Controller = hoodMotor.getClosedLoopController();
  
  
  @SuppressWarnings("removal")
  public shooter(int hoodMotor, int shootMotor1) {
    
    this.hoodMotor = new SparkMax(hoodMotor, MotorType.kBrushless);
    this.hoodMotorEncoder = this.hoodMotor.getAbsoluteEncoder();
    
    SparkMaxConfig config1 = new SparkMaxConfig();
    SparkMaxConfig config2 = new SparkMaxConfig();
    
    SparkMaxConfig configHood = new SparkMaxConfig();
    


    double kP = 0.0001; 
    double kF = 1.0 / 5676.0;
    config1
        .closedLoop
            .p(0)
            .i(0)
            .d(0)

            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .feedForward
                .kV(1)
                .kA(0)
                .kS(0);

    
    config2
        .inverted(true)
        .follow(shootMotor1);

      this.shootMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      shootMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    

  }

  public void SetRPM(int rpm) {
    sm1_Controller.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void RunIdle() {
    SetRPM(500);
  }

  public void RunFeeder() {
    feedControllerSrx.set(ControlMode.PercentOutput, 0.5);
  }

  public void setHoodMotorPosition(int setPoint) {
     //hoodMotor.setSetpoint(setPoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
