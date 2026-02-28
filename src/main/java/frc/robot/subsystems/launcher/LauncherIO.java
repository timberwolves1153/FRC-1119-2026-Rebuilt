package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {

  @AutoLog
  public static class LauncherInputs {
    public double leftLauncherAppliedVolts = 0;
    public double leftLauncherCurrentAmps = 0;
    public double leftLauncherVelovityRPM = 0;

    public double centerLauncherAppliedVolts = 0;
    public double centerLauncherCurrentAmps = 0;
    public double centerLauncherVelovityRPM = 0;

    public double rightLauncherAppliedVolts = 0;
    public double rightLauncherCurrentAmps = 0;
    public double rightLauncherVelovityRPM = 0;
  }

  public default void updateInputs(LauncherInputs inputs) {}

  public default void setLauncherVoltage(double volts) {}

  public default void setLauncherRPM(double rpm) {}

  public default void stopLauncher() {}

  public default boolean isVelocityInTolerance() {
    return true;
  }
}
