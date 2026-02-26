// Source code is decompiled from a .class file using FernFlower decompiler.
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeConstants;
import java.util.function.Consumer;

public class Intake extends SubsystemBase {
    SparkMax intake;
    SparkMax intakeRoller;
    SparkMaxConfig intakeConfig;
    RelativeEncoder intakeEncoder;
    SparkClosedLoopController intakeClosedLoopController;
    ShuffleboardTab intakeTab;
    GenericEntry intakeP;
    GenericEntry intakeG;
    GenericEntry intakeAngle;
    SysIdRoutine intakeRoutine;
    SysIdRoutine intakeRollerRoutine;

    public Command sysIdQuasistaticIntake(SysIdRoutine.Direction direction) {
        return this.intakeRoutine.quasistatic(direction);
    }

    public Command sysIdDynamicIntake(SysIdRoutine.Direction direction) {
        return this.intakeRoutine.dynamic(direction);
    }

    public Command sysIdQuasistaticIntakeRoller(SysIdRoutine.Direction direction) {
        return this.intakeRollerRoutine.quasistatic(direction);
    }

    public Command sysIdDynamicIntakeRoller(SysIdRoutine.Direction direction) {
        return this.intakeRollerRoutine.dynamic(direction);
    }

    public double getPosition() {
        return this.intakeEncoder.getPosition();
    }

    public void printPosition() {
        SmartDashboard.putNumber("Position", this.getPosition());
    }

    public void setPositionConversionFactor() {
        this.intakeConfig.encoder.positionConversionFactor(0.13392857142857142);
    }

    public void setIntakeAngle() {
        double intakeAngleValue = this.intakeAngle.getDouble(0.331);
        this.intakeClosedLoopController.setSetpoint(intakeAngleValue, ControlType.kPosition);
    }

    public void setIntake(double Setpoint) {
        this.intakeClosedLoopController.setSetpoint(Setpoint, ControlType.kPosition);
    }

    public void setSpeedIntakeRoller(double Speed) {
        this.intakeRoller.set(Speed);
    }

    public void stopIntakeRoller() {
        this.intakeRoller.set(0.0);
    }

    public void setSpeedIntake(double Speed) {
        this.intake.set(Speed);
    }

    public void stopIntake() {
        this.intake.set(0.0);
    }

    public void resetPosition() {
        this.intakeEncoder.setPosition(0.0);
    }

    public boolean getForwardSoftLimitReached() {
        return this.intake.getForwardSoftLimit().isReached();
    }

    public boolean getReversedSoftLimitReached() {
        return this.intake.getReverseSoftLimit().isReached();
    }

    public void setConstants() {
        double intakePValue = this.intakeP.getDouble(0.01);
        this.intakeConfig.closedLoop.pid(intakePValue, 0.0, 0.0);
        double intakeGValue = this.intakeG.getDouble(0.1);
        this.intakeConfig.closedLoop.feedForward.kG(intakeGValue);
        this.intake.configure(this.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Intake() {
        this.intake = new SparkMax(15, MotorType.kBrushless);
        this.intakeRoller = new SparkMax(14, MotorType.kBrushless);
        this.intakeConfig = new SparkMaxConfig();
        this.intakeEncoder = this.intake.getEncoder();
        this.intakeClosedLoopController = this.intake.getClosedLoopController();
        this.intakeTab = Shuffleboard.getTab("intakePID");
        this.intakeP = this.intakeTab.add("intakeP", 0).getEntry();
        this.intakeG = this.intakeTab.add("intakeG", 0).getEntry();
        this.intakeAngle = this.intakeTab.add("intakeAngle", 0).getEntry();
        this.intakeRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this.intake::setVoltage, (Consumer) null, this));
        this.intakeRollerRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(this.intakeRoller::setVoltage, (Consumer) null, this));
        this.intakeConfig.closedLoop.pid(IntakeConstants.INTAKE_PID_P, 0.0, 0.0);
        this.intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        this.intakeConfig
                .softLimit
                .forwardSoftLimit(0.331)
                .reverseSoftLimit(0.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        this.intakeConfig.idleMode(IdleMode.kBrake);
        this.resetPosition();
        this.setPositionConversionFactor();
        this.intakeConfig
                .closedLoop
                .feedForward
                .kG(IntakeConstants.INTAKE_FEEDFORWARD_G)
                .kS(0.0)
                .kV(IntakeConstants.INTAKE_SVA_V)
                .kA(0.0);
        this.intake.configure(this.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {
        this.printPosition();
    }
}
