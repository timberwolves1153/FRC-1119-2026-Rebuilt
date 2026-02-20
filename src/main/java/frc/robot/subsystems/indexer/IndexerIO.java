package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerInputs {
    public double IndexAppliedVolts = 0;
  }

  @AutoLog
  public static class LoadInputs {
    public double LoadAppliedVolts = 0;
  }

  public default void updateInputs(IndexerInputs inputs) {}

  public default void updateInputs(LoadInputs inputs) {}

  public default void setIndexVoltage(double volts) {}

  public default void setLoadVoltage(double volts) {}
}
