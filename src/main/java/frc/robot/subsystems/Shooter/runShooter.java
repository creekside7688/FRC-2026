package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterLookup;
import frc.robot.subsystems.drivebase.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runShooter extends Command {

    private final Shooter shooter;
    private final SwerveDrive sd;
    private double distance;
    /** Creates a new runShooter. */
    public runShooter(Shooter shooter, SwerveDrive sd) {
        this.shooter = shooter;
        this.sd = sd;
        addRequirements(shooter);
    }

    private void getDistanceToHub() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;

        distance = Math.hypot(
                (hubPose.getX() - sd.getPose().getX()),
                (hubPose.getY() - sd.getPose().getY()));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        getDistanceToHub();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        getDistanceToHub();
        double desiredRPM = ShooterLookup.lookupRPM(distance);
        shooter.SetRPM(desiredRPM);
        shooter.setHoodPosition(ShooterLookup.lookupAngle(distance));
        if (shooter.checkShooterRPMTolerance(desiredRPM)) {
            shooter.RunFeeder();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.RunIdle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
