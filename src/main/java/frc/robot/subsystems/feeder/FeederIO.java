package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederInputs {
    public double feederAppliedVolts = 0;
    public double feederCurrentAmps = 0;
  }

  public default void updateInputs(FeederInputs inputs) {}

  public default void setFeederVoltage(double volts) {}

  public default void stopFeeder() {}
}
