package frc.robot.interpolation;

import java.util.function.DoubleSupplier;

public class ShooterTable {

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ShooterMap =
      new InterpolatingTreeMap<>();

  static {
  }

  public DoubleSupplier getValueSupplier(double distance) {
    return () -> ShooterMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
