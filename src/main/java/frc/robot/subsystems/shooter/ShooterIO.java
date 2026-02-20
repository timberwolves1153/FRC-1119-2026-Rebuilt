package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    public double shooterAppliedVolts = 0;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setShooterVoltage(double volts) {}
}
