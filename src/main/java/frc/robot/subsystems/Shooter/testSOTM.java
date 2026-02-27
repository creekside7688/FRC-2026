// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterLookup;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.drivebase.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class testSOTM extends Command {
    /** Creates a new testVDS. */
    private final Shooter shooter;

    private final Feeder feeder;

    private final ShooterHood shooterHood;

    private final Spindexer spindexer;

    private final SwerveDrive sd;

    public testSOTM(Shooter shooter, Feeder feeder, ShooterHood shooterHood, Spindexer spindexer, SwerveDrive sd) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.shooterHood = shooterHood;
        this.spindexer = spindexer;
        this.sd = sd;
        addRequirements(shooter, feeder, shooterHood, spindexer);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = shooter.getVariableDistance();


        double distance1 = ShooterLookup.CalculationDelayOffset(sd.getPose(), sd.getChassisSpeeds().vxMetersPerSecond, sd.getChassisSpeeds().vyMetersPerSecond, 0.2).getX();
        

        /*shooter.SetRPM(desiredRPM);
        shooterHood.setHoodPosition(desiredAngle);
        if (shooter.checkShooterRPMTolerance() && shooterHood.checkShooterPositionTolerance()) {
            feeder.RunFeeder();
            spindexer.runIndexer();
        }*/

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
