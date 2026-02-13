package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Controller;
import frc.robot.subsystems.drivebase.SwerveDrive;

public class TeleopDrive extends Command {
    private final SwerveDrive drive;
    private final DoubleSupplier xSupplier;     
    private final DoubleSupplier ySupplier;     
    private final DoubleSupplier thetaSupplier;     

    private boolean isBlue;

    public TeleopDrive(SwerveDrive drive, Controller controller) {
        this.drive = drive;
        this.xSupplier = () -> controller.getLeftY() * -1;
        this.ySupplier = () -> controller.getLeftX() * -1;
        this.thetaSupplier = () -> controller.getRightX() * -1;

        addRequirements(drive);
    }

    public void initialize() {
        isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
    }
}
