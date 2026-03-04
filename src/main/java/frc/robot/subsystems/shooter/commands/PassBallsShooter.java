package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SpindexerConstants;
import frc.robot.subsystems.drivebase.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.ShooterHood;
import frc.robot.subsystems.spindexer.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassBallsShooter extends Command {

    private final Shooter shooter;
    private final SwerveDrive sd;
    private final ShooterFeeder feeder;
    private final ShooterHood shooterHood;
    private final Spindexer spindexer;
    private double distance;
    /** Creates a new PassBallsShooter. */
    public PassBallsShooter(
            Shooter shooter, SwerveDrive sd, ShooterFeeder feeder, ShooterHood shooterHood, Spindexer spindexer) {
        this.shooter = shooter;
        this.sd = sd;
        this.feeder = feeder;
        this.shooterHood = shooterHood;
        this.spindexer = spindexer;
        addRequirements(shooter, feeder, shooterHood, spindexer);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.SetRPM(4000);
        shooterHood.setHoodPosition(50);
        if (shooter.checkShooterRPMTolerance() && shooterHood.checkShooterPositionTolerance()) {
            feeder.RunFeeder();
            spindexer.setIndexer(SpindexerConstants.SPINDEXER_VOLTAGE);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.RunIdle();
        feeder.stopFeeder();
        spindexer.stopIndexer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
