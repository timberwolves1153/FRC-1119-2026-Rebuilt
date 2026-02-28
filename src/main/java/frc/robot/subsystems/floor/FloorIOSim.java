package frc.robot.subsystems.floor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FloorIOSim implements FloorIO {

  private DCMotorSim floorMotor;

  public FloorIOSim() {
    floorMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 1, 1),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void setFloorVoltage(double volts) {
    floorMotor.setInputVoltage(volts);
  }

  @Override
  public void stopFloor() {
    floorMotor.setInputVoltage(0);
  }

  @Override
  public void updateInputs(FloorInputs inputs) {
    floorMotor.update(0.02);
    inputs.floorAppliedVolts = floorMotor.getInputVoltage();
    inputs.floorCurrentAmps = floorMotor.getCurrentDrawAmps();
  }
}
