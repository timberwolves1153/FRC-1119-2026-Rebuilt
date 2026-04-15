package frc.robot.subsystems.floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Floor extends SubsystemBase {
  public FloorIO floorIO;
  public FloorInputsAutoLogged floorInputs;

  private final double FLOOR_SPEED = -12; // Kraken pulleys on base
  private final double FLOOR_REVERSE_SPEED = 12;
  private final double FLOOR_STOP_SPEED = 0;

  public Floor(FloorIO floorIO) {
    this.floorIO = floorIO;
    this.floorInputs = new FloorInputsAutoLogged();
  }

  public void setFloorVoltage(double volts) {
    floorIO.setFloorVoltage(volts);
  }

  public Command outTakeFloor() {
    return runOnce(() -> setFloorVoltage(FLOOR_REVERSE_SPEED));
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
    return runOnce(() -> setFloorVoltage(FLOOR_STOP_SPEED));
  }

  public Command floorCommand() {
    return startEnd(
        () -> {
          setFloorVoltage(FLOOR_SPEED);
        },
        () -> stopFloor());
  }
}
