package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {

  private DCMotorSim feederMotor;

  public FeederIOSim() {
    feederMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(2), 1, 1),
            DCMotor.getFalcon500(2));
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederMotor.setInputVoltage(volts);
  }

  @Override
  public void stopFeeder() {
    feederMotor.setInputVoltage(0);
  }

  @Override
  public void updateInputs(FeederInputs inputs) {
    feederMotor.update(0.02);
    inputs.feederAppliedVolts = feederMotor.getInputVoltage();
    inputs.feederCurrentAmps = feederMotor.getCurrentDrawAmps();
  }
}
