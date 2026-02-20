package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim deployMotor;
  private DCMotorSim intakeMotor;

  public IntakeIOSim() {
    deployMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    intakeMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotor.setInputVoltage(volts);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    deployMotor.update(0.02);
    intakeMotor.update(0.02);

    inputs.deployAppliedVolts = deployMotor.getInputVoltage();
    inputs.intakeAppliedVolts = intakeMotor.getInputVoltage();
  }
}
