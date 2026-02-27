package frc.robot.subsystems.shooter.shooterCommands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterLookup;
import frc.robot.subsystems.drivebase.SwerveDrive;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooter extends Command {

    private final Shooter shooter;
    private final SwerveDrive sd;
    private final Feeder feeder;
    private final ShooterHood shooterHood;
    private double distance;
    /** Creates a new runShooter. */
    public RunShooter(Shooter shooter, SwerveDrive sd, Feeder feeder, ShooterHood shooterHood) {
        this.shooter = shooter;
        this.sd = sd;
        this.feeder = feeder;
        this.shooterHood = shooterHood;
        addRequirements(shooter, feeder, shooterHood);
    }

    private void getDistanceToHub() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        // var hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;
        var hubPose = GameConstants.HUB_RED;

        // distance = Math.hypot(
        // (hubPose.getX() - sd.getPose().getX()),
        // (hubPose.getY() - sd.getPose().getY()5));

        distance =
                hubPose.getDistance(sd.getPose().getTranslation().minus(new Translation2d(Inches.of(5), Inches.of(6))));

        SmartDashboard.putNumber("distance from hub", distance);
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
        shooterHood.setHoodPosition(ShooterLookup.lookupAngle(distance));
        if (shooter.checkShooterRPMTolerance() && shooterHood.checkShooterPositionTolerance()) {
            feeder.RunFeeder();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.RunIdle();
        feeder.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
