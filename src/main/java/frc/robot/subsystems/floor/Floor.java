package frc.robot.subsystems.floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Floor extends SubsystemBase {
  public FloorIO floorIO;
  public FloorInputsAutoLogged floorInputs;

  private final double FLOOR_SPEED = -12; // Kraken pulleys on base

  public Floor(FloorIO floorIO) {
    this.floorIO = floorIO;
    this.floorInputs = new FloorInputsAutoLogged();
  }

  public void setFloorVoltage(double volts) {
    floorIO.setFloorVoltage(volts);
  }

  public void stopFloor() {
    floorIO.stopFloor();
  }

  @Override
  public void periodic() {
    floorIO.updateInputs(floorInputs);
    Logger.processInputs("floor", floorInputs);
  }

  public Command floorStopCommand() {
    return runOnce(() -> setFloorVoltage(0));
  }

  public Command floorCommand() {
    return startEnd(
        () -> {
          setFloorVoltage(FLOOR_SPEED);
        },
        () -> stopFloor());
  }
}
