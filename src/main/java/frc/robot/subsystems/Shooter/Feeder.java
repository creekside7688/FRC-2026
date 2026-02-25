package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Feeder extends SubsystemBase {
    private final TalonSRX feedControllerSrx = new TalonSRX(ShooterConstants.FEED_MOTOR_SRX_ID);

    public Feeder() {}

    public void RunFeeder() {
        feedControllerSrx.set(ControlMode.PercentOutput, ShooterConstants.RUN_FEEDER_OUTPUT);
    }

    public void stopFeeder() {
        feedControllerSrx.set(ControlMode.PercentOutput, 0);
    }

    public void periodic() {}

    public Command runShooterFeeder() {
        return this.runOnce(() -> this.RunFeeder());
    }

    public Command stopShooterFeeder() {
        return this.runOnce(() -> this.stopFeeder());
    }
}
