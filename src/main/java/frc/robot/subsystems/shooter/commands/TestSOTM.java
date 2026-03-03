// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.ShooterLookup;
import frc.robot.subsystems.drivebase.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.ShooterHood;
import frc.robot.subsystems.spindexer.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class TestSOTM extends Command {
    /** Creates a new testVDS. */
    private final Shooter shooter;

    private final ShooterFeeder feeder;

    private final ShooterHood shooterHood;

    private final Spindexer spindexer;

    private final SwerveDrive sd;

    private final Supplier<Double> getControllerX;

    private final Supplier<Double> getControllerY;

    public TestSOTM(
            Shooter shooter, ShooterFeeder feeder, ShooterHood shooterHood, Spindexer spindexer, SwerveDrive sd, Supplier<Double> getControllerX, Supplier<Double> getControllerY) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.shooterHood = shooterHood;
        this.spindexer = spindexer;
        this.sd = sd;
        this.getControllerX = getControllerX;
        this.getControllerY = getControllerY;
        addRequirements(shooter, feeder, shooterHood, spindexer, sd);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = shooter.getVariableDistance();
        double angle = sd.getPose().getRotation().getDegrees();

        angle = angle + ShooterLookup.CalculationDelayOffset(
                        sd.getPose(),
                        sd.getChassisSpeeds().vxMetersPerSecond,
                        sd.getChassisSpeeds().vyMetersPerSecond,
                        0.2)
                .getY();
        

        distance = distance + ShooterLookup.CalculationDelayOffset(
                        sd.getPose(),
                        sd.getChassisSpeeds().vxMetersPerSecond,
                        sd.getChassisSpeeds().vyMetersPerSecond,
                        0.2)
                .getX();
        
        final double finalDistance = distance + ShooterLookup.RecurssiveFOT(sd.getChassisSpeeds().vxMetersPerSecond, sd.getChassisSpeeds().vyMetersPerSecond, shooterHood.getHoodPosition(), sd.getPose().getRotation().getDegrees(), distance).getX();
        
        final double finalAngle = angle + ShooterLookup.RecurssiveFOT(sd.getChassisSpeeds().vxMetersPerSecond, sd.getChassisSpeeds().vyMetersPerSecond, shooterHood.getHoodPosition(), sd.getPose().getRotation().getDegrees(), distance).getY();
        
        double desiredRPM = ShooterLookup.lookupRPM(finalDistance);
        double desiredAngle = ShooterLookup.lookupAngle(finalDistance);

        shooter.SetRPM(desiredRPM);
        shooterHood.setHoodPosition(desiredAngle);
        if (shooter.checkShooterRPMTolerance() && shooterHood.checkShooterPositionTolerance()) {
            feeder.RunFeeder();
            spindexer.runIndexer();
        }

        DriveCommands.joystickDriveWithRotationalOverride(sd, () -> getControllerX.get(), () -> getControllerY.get(), () -> Rotation2d.fromRadians(finalAngle));

        

        // shooter.setHoodPosition(ShooterLookup.lookupAngle(desiredAngle));
        /*

        if (shooter.checkShooterRPMTolerance(desiredRPM) && shooter.checkShooterPositionTolerance(desiredAngle)) {
            shooter.RunFeeder();
        }
         */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotor1Voltage(0);
        spindexer.stopIndexer();
        feeder.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
