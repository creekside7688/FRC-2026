package frc.robot.subsystems;
import frc.robot.constants.ModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.ModuleConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkClosedLoopController drivePIDController;
    private final SparkClosedLoopController turnPIDController;

    private double angularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveMotor, int turnMotor, double angularOffset) {
        this.driveMotor = new SparkMax(driveMotor, MotorType.kBrushless);
        this.turnMotor = new SparkMax(turnMotor, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        turnConfig = new SparkMaxConfig();
        

        /*
        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.restoreFactoryDefaults();*/

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = this.turnMotor.getAbsoluteEncoder();

        this.drivePIDController = this.driveMotor.getClosedLoopController();
        this.turnPIDController = this.turnMotor.getClosedLoopController();
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        /*this.drivePIDController.setFeedbackDevice(driveEncoder);
        this.turnPIDController.setFeedbackDevice(turnEncoder);*/

        driveConfig.encoder.positionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);
        /*this.driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_FACTOR);
        this.driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);*/

        turnConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.TURN_ENCODER_POSITION_FACTOR);
        turnConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);
        /*this.turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POSITION_FACTOR);
        this.turnEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);*/

        // Invert the encoder because the output shaft rotates the opposite direction in the modules.
        turnConfig.absoluteEncoder.inverted(ModuleConstants.TURN_ENCODER_INVERTED);
        //this.turnEncoder.setInverted(ModuleConstants.TURN_ENCODER_INVERTED);
        
        /*
         * Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees will go
         * through 0 rather than the other direction which is a longer route.
         */
        
        turnConfig.closedLoop.positionWrappingEnabled(true);
        turnConfig.closedLoop.positionWrappingMinInput(ModuleConstants.TURN_PID_MINIMUM_INPUT);
        turnConfig.closedLoop.positionWrappingMaxInput(ModuleConstants.TURN_PID_MAXIMUM_INPUT);
        /*  this.turnPIDController.setPositionPIDWrappingEnabled(true);
        this.turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_PID_MINIMUM_INPUT);
        this.turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_PID_MAXIMUM_INPUT);*/

        driveConfig.closedLoop.p(ModuleConstants.DRIVE_P);
        driveConfig.closedLoop.i(ModuleConstants.DRIVE_I);
        driveConfig.closedLoop.d(ModuleConstants.DRIVE_D);
        driveConfig.closedLoop.velocityFF(ModuleConstants.DRIVE_FF);
        driveConfig.closedLoop.outputRange(ModuleConstants.DRIVE_MINIMUM_OUTPUT, ModuleConstants.DRIVE_MAXIMUM_OUTPUT);
        /* 
        this.drivePIDController.setP(ModuleConstants.DRIVE_P);
        this.drivePIDController.setI(ModuleConstants.DRIVE_I);
        this.drivePIDController.setD(ModuleConstants.DRIVE_D);
        this.drivePIDController.setFF(ModuleConstants.DRIVE_FF);
        this.drivePIDController.setOutputRange(ModuleConstants.DRIVE_MINIMUM_OUTPUT, ModuleConstants.DRIVE_MAXIMUM_OUTPUT);*/

        turnConfig.closedLoop.p(ModuleConstants.TURN_P);
        turnConfig.closedLoop.i(ModuleConstants.TURN_I);
        turnConfig.closedLoop.d(ModuleConstants.TURN_D);
        turnConfig.closedLoop.velocityFF(ModuleConstants.TURN_FF);
        turnConfig.closedLoop.outputRange(ModuleConstants.TURN_MINIMUM_OUTPUT, ModuleConstants.TURN_MAXIMUM_OUTPUT);

        /*
        this.turnPIDController.setP(ModuleConstants.TURN_P);
        this.turnPIDController.setI(ModuleConstants.TURN_I);
        this.turnPIDController.setD(ModuleConstants.TURN_D);
        this.turnPIDController.setFF(ModuleConstants.TURN_FF);
        this.turnPIDController.setOutputRange(ModuleConstants.TURN_MINIMUM_OUTPUT, ModuleConstants.TURN_MAXIMUM_OUTPUT);*/

        driveConfig.idleMode(ModuleConstants.DRIVE_IDLE_MODE);
        turnConfig.idleMode(ModuleConstants.TURN_IDLE_MODE);
        driveConfig.smartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        turnConfig.smartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);
        /*
        this.driveMotor.setIdleMode(ModuleConstants.DRIVE_IDLE_MODE);
        this.turnMotor.setIdleMode(ModuleConstants.TURN_IDLE_MODE);
        this.driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        this.turnMotor.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);*/

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set the angular offset of the module.
        this.angularOffset = angularOffset;
        this.desiredState.angle = new Rotation2d(turnEncoder.getPosition());
        this.driveEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(turnEncoder.getPosition() - angularOffset)
        );
    }

    /**
     * Returns the current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(turnEncoder.getPosition() - angularOffset)
        );
    }

    /**
     * Returns the desired state of the module.
     * @return The desired state of the module.
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;

    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(angularOffset))
        );

        // Optimize to prevent having to turn more than 90 degrees.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            correctedState,
            new Rotation2d(turnEncoder.getPosition())
        );

        // Drive towards setpoints.
        drivePIDController.setReference(
            optimizedState.speedMetersPerSecond,
            SparkMax.ControlType.kVelocity
        );

        turnPIDController.setReference(
            optimizedState.angle.getRadians(),
            SparkMax.ControlType.kPosition
        );

        this.desiredState = desiredState;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
}
