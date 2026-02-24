// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    SparkMax intake = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    SparkMax intakeRoller = new SparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    RelativeEncoder intakeEncoder = intake.getEncoder();
    SparkClosedLoopController intakeClosedLoopController = intake.getClosedLoopController();
    SysIdRoutine intakeRoutine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(intake::setVoltage, null, this));
    SysIdRoutine intakeRollerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(intakeRoller::setVoltage, null, this));

    public Command sysIdQuasistaticIntake(SysIdRoutine.Direction direction) {
        return intakeRoutine.quasistatic(direction);
    }

    public Command sysIdDynamicIntake(SysIdRoutine.Direction direction) {
        return intakeRoutine.dynamic(direction);
    }

    public Command sysIdQuasistaticIntakeRoller(SysIdRoutine.Direction direction) {
        return intakeRollerRoutine.quasistatic(direction);
    }

    public Command sysIdDynamicIntakeRoller(SysIdRoutine.Direction direction) {
        return intakeRollerRoutine.dynamic(direction);
    }

    public double getPosition() {
        return intakeEncoder.getPosition();
    }

    public void printPosition() {
        SmartDashboard.putNumber("Position", getPosition());
    }

    public void setPositionConversionFactor() {
        intakeConfig.encoder.positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);
    }

    public void goToAngle(double Angle) {
        intakeClosedLoopController.setSetpoint(Angle, ControlType.kPosition);
    }

    public void setSpeedIntakeRoller(double Speed) {
        intakeRoller.set(Speed);
    }

    public void stopIntakeRoller() {
        intakeRoller.set(0);
    }

    public void setSpeedIntake(double Speed) {
        intake.set(Speed);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void resetPosition() {
        intakeEncoder.setPosition(0);
    }

    public boolean getForwardSoftLimitReached() {
        return intake.getForwardSoftLimit().isReached();
    }

    public boolean getReversedSoftLimitReached() {
        return intake.getReverseSoftLimit().isReached();
    }
    /** Creates a new Intake. */
    public Intake() {
        intakeConfig.closedLoop.pid(
                IntakeConstants.INTAKE_PID_P, IntakeConstants.INTAKE_PID_I, IntakeConstants.INTAKE_PID_D);
        intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeConfig
                .softLimit
                .forwardSoftLimit(IntakeConstants.INTAKE_SOFT_FORWARD_LIMIT)
                .reverseSoftLimit(IntakeConstants.INTAKE_SOFT_REVERSED_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);

        intakeConfig.idleMode(IdleMode.kBrake);
        resetPosition();
        setPositionConversionFactor();
        intakeConfig
                .closedLoop
                .feedForward
                .kS(IntakeConstants.INTAKE_SVA_S)
                .kV(IntakeConstants.INTAKE_SVA_V)
                .kA(IntakeConstants.INTAKE_SVA_A);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
