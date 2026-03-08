// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GameConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShooterLookup;
import java.util.function.Supplier;

public class ShooterHood extends SubsystemBase {
    /** Creates a new ShooterHood. */
    private SparkMaxConfig configHood;

    private final SparkMax hoodMotor;

    private final AbsoluteEncoder hoodMotorEncoder;

    private ShuffleboardTab tab = Shuffleboard.getTab("shooter");

    private GenericEntry hoodVoltage = tab.add("hoodVoltage", 0).getEntry();

    private GenericEntry hoodPos = tab.add("hoodPos", 60).getEntry();

    double desiredAngle = 0;

    private final SparkClosedLoopController hood_Controller;

    public ShooterHood() {

        this.hoodMotor = new SparkMax(HoodConstants.BALL_HOOD_MOTOR, MotorType.kBrushless);
        this.hoodMotorEncoder = this.hoodMotor.getAbsoluteEncoder();
        hood_Controller = this.hoodMotor.getClosedLoopController();

        configHood = new SparkMaxConfig();

        configHood.closedLoop.p(HoodConstants.HOOD_P).i(0).d(0).outputRange(-0.2, 0.2);
        configHood.smartCurrentLimit(HoodConstants.STALL_CURRENT_LIMIT, HoodConstants.FREE_CURRENT_LIMIT);

        configHood.encoder.positionConversionFactor(HoodConstants.ANGLECHANGE_PER_ROTATION);

        // Soft Limits: Prevent the hood from slamming into the frame
        configHood
                .softLimit
                .forwardSoftLimit(75) // Max angle
                .reverseSoftLimit(45) // Min angle
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);

        configHood.idleMode(IdleMode.kBrake);

        hoodMotor.getEncoder().setPosition(75);

        hoodMotor.configure(configHood, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setHoodPosition(double setPoint) {
        desiredAngle = setPoint;
        hood_Controller.setSetpoint(setPoint, ControlType.kPosition);
    }

    public void setVariableHoodPosition() {
        int setPoint = (int) (hoodPos.getDouble(0));
        setHoodPosition(setPoint);
    }

    public boolean checkShooterPositionTolerance() {
        double shooterAngle = hoodMotor.getEncoder().getPosition();
        double toleranceHigh = desiredAngle * 1.005;
        double toleranceLow = desiredAngle * 0.995;
        if (shooterAngle > toleranceLow && shooterAngle < toleranceHigh) {
            return true;
        }
        return false;
    }

    public boolean checkVariablePositionTolerance() {
        double targetAngle = hoodPos.getDouble(0);
        double shooterAngle = hoodMotor.getEncoder().getPosition();
        double toleranceHigh = targetAngle * 1.02;
        double toleranceLow = targetAngle * 0.98;
        if (shooterAngle > toleranceLow && shooterAngle < toleranceHigh) {
            return true;
        }
        return false;
    }

    // Hood motor Voltages

    public void setHoodMotorVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    public void setVariableHMVoltage(boolean inverted) {
        double hoodMVolts = hoodVoltage.getDouble(0);
        if (inverted) hoodMVolts *= -1;
        setHoodMotorVoltage(hoodMVolts);
    }

    public boolean underTrench(Pose2d position) {
        for (int i = 0; i < FieldConstants.TRENCH_ZONES_X.length; i++) {

            if (position.getX() > FieldConstants.TRENCH_ZONES_X[i][0]
                    && position.getX() < FieldConstants.TRENCH_ZONES_X[i][1]) {

                if (position.getY() > FieldConstants.TRENCH_ZONES_Y[i][0]
                        && position.getY() < FieldConstants.TRENCH_ZONES_Y[i][1]) return true;
            }
        }
        return false;
    }

    public void hoodUnderTrench(Pose2d position) {
        if (underTrench(position)) {
            setHoodPosition(75);
        }
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Encoder position:", hoodMotor.getEncoder().getPosition());
        // SmartDashboard.putBoolean("Within tolerance?", checkVariablePositionTolerance());
        // This method will be called once per scheduler run
    }

    private double getAngleFromPose(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;

        return Math.hypot(hubPose.getX() - pose.getX(), hubPose.getY() - pose.getY());
    }

    public Command setShooterAngle(Supplier<Pose2d> poseSupplier) {
        // implicitly requires `this`
        return this.runOnce(
                () -> this.setHoodPosition(ShooterLookup.lookupAngle(getAngleFromPose(poseSupplier.get()))));
    }

    public Command runHoodMotor(double voltage) {
        return this.runOnce(() -> hoodMotor.setVoltage(voltage));
    }
}
