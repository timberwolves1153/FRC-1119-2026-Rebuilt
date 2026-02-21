package frc.robot.interpolation;

import java.util.function.DoubleSupplier;

public class LauncherTable {

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap =
      new InterpolatingTreeMap<>();

  static {
  }

  public DoubleSupplier getLaunchVoltageSupplier(double distance) {
    return () -> launcherMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
