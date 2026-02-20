package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {

  private DCMotorSim indexMotor;
  private DCMotorSim loadMotor;

  public IndexerIOSim() {
    indexMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 1, 1),
            DCMotor.getNeoVortex(1));

    loadMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(2), 1, 1),
            DCMotor.getFalcon500(2));
  }

  @Override
  public void setIndexVoltage(double volts) {
    indexMotor.setInputVoltage(volts);
  }

  @Override
  public void setLoadVoltage(double volts) {
    indexMotor.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    indexMotor.update(0.02);
    inputs.IndexAppliedVolts = indexMotor.getInputVoltage();
  }

  @Override
  public void updateInputs(LoadInputs inputs) {
    loadMotor.update(0.02);
    inputs.LoadAppliedVolts = loadMotor.getInputVoltage();
  }
}
