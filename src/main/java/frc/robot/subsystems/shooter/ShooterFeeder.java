package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;

public class ShooterFeeder extends SubsystemBase {
    private final TalonSRX feedControllerSrx = new TalonSRX(FeederConstants.FEED_MOTOR_SRX_ID);

    public ShooterFeeder() {}

    public void RunFeeder() {
        feedControllerSrx.set(ControlMode.PercentOutput, FeederConstants.RUN_FEEDER_OUTPUT);
    }

    public void UnclogFeeder() {
        feedControllerSrx.set(ControlMode.PercentOutput, -FeederConstants.RUN_FEEDER_OUTPUT);
        
    }

    public void stopFeeder() {
        feedControllerSrx.set(ControlMode.PercentOutput, 0);
    }

    public void periodic() {}

    public Command runShooterFeeder() {
        return this.runOnce(() -> this.RunFeeder());
    }

    public Command unclogShooterFeeder() {
        return this.runOnce(() -> this.UnclogFeeder());
    }

    public Command stopShooterFeeder() {
        return this.runOnce(() -> this.stopFeeder());
    }
}
