package frc.robot.subsystems.floor;

import org.littletonrobotics.junction.AutoLog;

public interface FloorIO {

  @AutoLog
  public static class FloorInputs {
    public double floorAppliedVolts = 0;
    public double floorCurrentAmps = 0;
  }

  public default void updateInputs(FloorInputs inputs) {}

  public default void setFloorVoltage(double volts) {}

  public default void stopFloor() {}
}
