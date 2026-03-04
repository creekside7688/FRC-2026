// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterLookup;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    /** Creates a new shooter. */
    private SparkMaxConfig config1;

    private SparkMaxConfig config2;
    private SparkMaxConfig configHood;

    private ShuffleboardTab tab = Shuffleboard.getTab("shooter");

    // private GenericEntry voltage = tab.add("shooterVoltage", 0).getEntry();

    private GenericEntry shootRPM = tab.add("shootRPM", 0).getEntry();

    private GenericEntry desiredDistance = tab.add("desiredDistance", 100).getEntry();

    private final SparkMax shootMotor1 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID1, MotorType.kBrushless);
    private final SparkMax shootMotor2 = new SparkMax(ShooterConstants.BALL_SHOOTING_MOTOR_ID2, MotorType.kBrushless);

    double desiredRPM = 0;

    private final AbsoluteEncoder shootMotor1Encoder;

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(shootMotor1::setVoltage, null, this));

    private final SparkClosedLoopController sm1_Controller;

    public Shooter() {

        ShooterLookup.initializeTable();
        SmartDashboard.putBoolean("t", false);

        this.shootMotor1Encoder = shootMotor1.getAbsoluteEncoder();

        sm1_Controller = this.shootMotor1.getClosedLoopController();

        config1 = new SparkMaxConfig();
        config2 = new SparkMaxConfig();

        config1.encoder.uvwAverageDepth(2);
        config2.encoder.uvwAverageDepth(2);

        config1.encoder.uvwMeasurementPeriod(16);
        config2.encoder.uvwMeasurementPeriod(16);

        config1.closedLoop
                .p(ShooterConstants.SHOOTER_P)
                .i(ShooterConstants.SHOOTER_I)
                .d(ShooterConstants.SHOOTER_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .feedForward
                .kV(ShooterConstants.SHOOTER_KV)
                .kA(ShooterConstants.SHOOTER_KA)
                .kS(ShooterConstants.SHOOTER_KS);

        config1.encoder.velocityConversionFactor(1);
        config1.signals.primaryEncoderVelocityAlwaysOn(true);
        config1.signals.primaryEncoderPositionAlwaysOn(true);

        config1.smartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT);
        config2.smartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT);

        config1.idleMode(IdleMode.kCoast);
        config2.idleMode(IdleMode.kCoast);

        config2.follow(shootMotor1, true);

        this.shootMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shootMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getVariableDistance() {
        return desiredDistance.getDouble(100);
    }
    // flywheel

    public boolean checkShooterRPMTolerance() {
        double shooterVelocity = shootMotor1.getEncoder().getVelocity();
        double toleranceHigh = desiredRPM * 1.035;
        double toleranceLow = desiredRPM * 0.965;
        if (shooterVelocity > toleranceLow && shooterVelocity < toleranceHigh) {
            return true;
        }
        return false;
    }

    public boolean checkVariableRPMTolerance() {
        double targetRPM = shootRPM.getDouble(0);
        double shooterVelocity = shootMotor1.getEncoder().getVelocity();
        double toleranceHigh = targetRPM * 1.035;
        double toleranceLow = targetRPM * 0.965;
        if (shooterVelocity > toleranceLow && shooterVelocity < toleranceHigh) {
            return true;
        }
        return false;
    }

    public void SetRPM(double rpm) {
        desiredRPM = rpm;
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

    /*
    public void runVariableVoltage() {
        SmartDashboard.putBoolean("t", true);
        double retrievedVoltage = voltage.getDouble(0);
        shootMotor1.setVoltage(retrievedVoltage);
        SmartDashboard.putNumber("set voltage", retrievedVoltage);
    }
        */

    // feeder

    @Override
    public void periodic() {
        // This method will be called once per scheduler run;
        SmartDashboard.putNumber("Shooter 1 Velocity", shootMotor1.getEncoder().getVelocity());
        shootMotor1.getEncoder().getVelocity();
        shootMotor1.getEncoder().getPosition();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    private double getRPMFromPose(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;

        return Math.hypot(hubPose.getX() - pose.getX(), hubPose.getY() - pose.getY());
    }

    public Command setShooterRPM(Supplier<Pose2d> poseSupplier) {
        // implicitly requires `this`
        return this.runOnce(() -> this.SetRPM(ShooterLookup.lookupRPM(getRPMFromPose(poseSupplier.get()))));
    }

    public Command setSpecificShooterRPM(int rpm) {
        // implicitly requires `this`
        return this.runOnce(() -> this.SetRPM(rpm));
    }

    public Command runQuickShooter() {
        return this.runOnce(() -> this.shootMotor1.set(0.4));
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
}
