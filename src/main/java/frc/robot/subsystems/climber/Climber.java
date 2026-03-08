// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {
    public boolean focus = true;
    public boolean test;
    public final SparkMax largeHookMotor = new SparkMax(ClimberConstants.LARGE_HOOK_MOTOR_ID, MotorType.kBrushless);
    // private final SparkMax smallHookMotor = new SparkMax(ClimberConstants.SMALL_HOOK_MOTOR_ID, MotorType.kBrushless);
    // private final SparkMax gapBlockerMotor = new SparkMax(ClimberConstants.GAP_BLOCKER_MOTOR_ID,
    // MotorType.kBrushless);

    private final RelativeEncoder largeHookEncoder = largeHookMotor.getEncoder();
    // private ShuffleboardTab tab = Shuffleboard.getTab("climber");

    // private final RelativeEncoder smallHookEncoder = smallHookMotor.getAlternateEncoder();
    // private final RelativeEncoder gapBlockerEncoder = gapBlockerMotor.getAlternateEncoder();

    private final SparkMaxConfig largeHookConfig = new SparkMaxConfig();
    // private final SparkMaxConfig smallHookConfig = new SparkMaxConfig();

    // private double targetGapBlockerPosition = 0;
    private double targetLargeHookPosition = 0;
    // private double targetSmallHookPosition = 0;

    /** Creates a new Climber. */
    public Climber(boolean is_a_test) {
        this.test = is_a_test;

        if (this.test) {
            this.focus = false;
        }

        // largeHookConfig.inverted(true);
        largeHookConfig.smartCurrentLimit(ClimberConstants.STALL_CURRENT_LIMIT);
        largeHookConfig.idleMode(IdleMode.kBrake);
        largeHookConfig.encoder.positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);
        largeHookConfig.closedLoop.positionWrappingEnabled(true);
        largeHookConfig.closedLoop.positionWrappingInputRange(0, ClimberConstants.CHAIN_LOOP_LENGTH_INCHES);

        // smallHookConfig.inverted(ClimberConstants.SMALL_HOOK_CLOCKWISE);
        // smallHookConfig.encoder.positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);
        // smallHookConfig.closedLoop.positionWrappingEnabled(true);
        // smallHookConfig.closedLoop.positionWrappingInputRange(0, ClimberConstants.CHAIN_LOOP_LENGTH_INCHES);

        largeHookMotor.configure(largeHookConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // smallHookMotor.configure(smallHookConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // gapBlockerEncoder.setPosition(0); // assuming the gap blocker starts open
        // smallHookEncoder.setPosition(0); // assuming both hooks start backed
        this.zeroEncoder(); // assuming both hooks start backed
    }

    public void zeroEncoder() {
        // gapBlockerEncoder.setPosition(0);
        // smallHookEncoder.setPosition(0);
        largeHookEncoder.setPosition(0);
    }
    // public void preStart() {
    //     // this.noGap();
    //     this.largeHookReset();
    //     // this.smallHookReset();
    //     // gapBlockerMotor.set(0);
    //     // smallHookMotor.set(0);
    // }

    // public void gap() {
    //     gapBlockerMotor.set(ClimberConstants.GAP_BLOCKER_MOTOR_SPEED);
    // }

    public void largeHookZero() {
        if (!test) {
            focus = false;
            targetLargeHookPosition = 0;
        }
    }

    public void setTestMode(boolean is_test_mode) {
        test = is_test_mode;
        if (is_test_mode) {
            focus = false;
        }
    }

    public void largeHookPre() {
        if (test) {
            largeHookMotor.set(ClimberConstants.CLIMBER_TEST_SPEED);
        } else {
            focus = false;
            targetLargeHookPosition = ClimberConstants.HOOK_PRE_TARGET_POSITION_INCHES;
        }
    }

    public void largeHookPost() {
        if (test) {
            largeHookMotor.set(-ClimberConstants.CLIMBER_TEST_SPEED);
        } else {
            focus = false;
            targetLargeHookPosition = ClimberConstants.HOOK_POST_TARGET_POSITION_INCHES;
        }
    }

    // public void largeHookPull() {
    //     focus = false;
    //     targetLargeHookPosition =
    //             ClimberConstants.HOOK_RESET_TARGET_POSITION_INCHES +
    // ClimberConstants.HOOK_PULL_TARGET_POSITION_INCHES;
    // }

    // public void smallHookPre() {
    //     targetSmallHookPosition = ClimberConstants.CHAIN_CIRCUMFERENCE_INCHES * 3 / 8; // 45 deg from hook point)
    // }

    // public void smallHookPost() {
    //     targetSmallHookPosition = ClimberConstants.CHAIN_CIRCUMFERENCE_INCHES * .5
    //             + ClimberConstants.HOOK_POST_PERCENT
    //                     * ClimberConstants.CHAIN_CENTERS_DISTANCE_INCHES; // a little after horizontal level
    // }

    // public void smallHookPull() {
    //     targetSmallHookPosition = ClimberConstants.CHAIN_CIRCUMFERENCE_INCHES * .5
    //             + ClimberConstants.HOOK_PULL_PERCENT
    //                     * ClimberConstants.CHAIN_CENTERS_DISTANCE_INCHES; // pull down the chain
    // }

    // public void noGap() {
    //     targetGapBlockerPosition = 0;
    // }

    public void largeHookReset() {
        focus = false;
        targetLargeHookPosition = 0;
    }

    // public void smallHookReset() {
    //     targetSmallHookPosition = 0;
    // }

    private void setMotor(SparkMax motor, double currentPos, double targetPos) {
        double diff = currentPos - targetPos;
        // motor.set(speed * Math.max(-1, Math.min(1, diff / accuracy)));
        if (Math.abs(diff) <= ClimberConstants.HOOK_FOCUS_ACCURACY_INCHES) {
            motor.set(0);
            focus = true;
        } else if (Math.abs(diff) <= ClimberConstants.HOOK_ACCURACY_INCHES) {
            motor.set(ClimberConstants.HOOK_MOTOR_SPEED
                    * -1
                    *
                    // ClimberConstants.SMALL_HOOK_CLOCKWISE *
                    (diff / ClimberConstants.HOOK_ACCURACY_INCHES));
        } else if (Math.abs(diff) > ClimberConstants.HOOK_ACCURACY_INCHES) {
            motor.set(ClimberConstants.HOOK_MOTOR_SPEED
                    * -1
                    *
                    // ClimberConstants.SMALL_HOOK_CLOCKWISE *
                    Math.signum(diff));
        }
    }

    public void testIfEnd() {
        largeHookMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // setMotor(
        //         gapBlockerMotor,
        //         gapBlockerEncoder.getPosition(),
        //         targetGapBlockerPosition,
        //         ClimberConstants.GAP_BLOCKER_ACCURACY_ROTATIONS,
        //         ClimberConstants.GAP_BLOCKER_MOTOR_SPEED * (ClimberConstants.GAP_BLOCKER_CLOCKWISE ? -1 : 1));

        if (!test) {
            setMotor(largeHookMotor, largeHookEncoder.getPosition(), targetLargeHookPosition);
        }
        // SmartDashboard.putNumber("climber encoder pos", largeHookEncoder.getPosition());
        // SmartDashboard.putBoolean("climberTestMode", test);

        // setMotor(
        //         smallHookMotor,
        //         smallHookEncoder.getPosition(),
        //         targetSmallHookPosition,
        //         ClimberConstants.HOOK_ACCURACY_INCHES,
        //         ClimberConstants.HOOK_MOTOR_SPEED);
    }

    public Command getSequence(Command climberReset, Command climberPre, Command climberPost) {
        return test ? new InstantCommand() : new SequentialCommandGroup(climberReset, climberPre, climberPost);
    }
}
