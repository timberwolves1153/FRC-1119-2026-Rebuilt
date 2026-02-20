package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double deployAppliedVolts = 0;
    public double intakeAppliedVolts = 0;
    public double retractAppliedVolts = 0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setDeployVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}
}
