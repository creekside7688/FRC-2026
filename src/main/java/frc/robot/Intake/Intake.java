// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  SparkMax intake = new SparkMax(Constants.Intake_MotorID, MotorType.kBrushless);
  SparkMaxConfig intake_config = new SparkMaxConfig();
  SparkClosedLoopController intake_closedloopcontroller = intake.getClosedLoopController();
  TalonSRX intake_TalonSRX = new TalonSRX(Constants.Intake_TalonSRXID);

  public void SetFeedBack() {

  }

  public void SetTalonSRXOutput() {
    intake_TalonSRX.set(ControlMode.PercentOutput, Constants.Intake_TalonSRX_OutputValue);
  }

  public void StopTalonSRX() {
    intake_TalonSRX.set(ControlMode.PercentOutput, 0);
  }

  public void GoToAngle(double Angle) {
    intake_closedloopcontroller.setSetpoint(Angle, ControlType.kPosition);
  }

  public void SetSpeed(double Speed) {
    intake.set(Speed);
  }

  public void Stop() {
    intake.set(0);
  }

  /** Creates a new Intake. */
  public Intake() {
    intake_config.closedLoop.pid(Constants.Intake_PID_P, Constants.Intake_PID_I, Constants.Intake_PID_D);
    intake_config.closedLoop.feedForward
    .kS(Constants.Intake_SVA_S)
    .kV(Constants.Intake_SVA_V)
    .kA(Constants.Intake_SVA_A);
    intake.configure(intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
